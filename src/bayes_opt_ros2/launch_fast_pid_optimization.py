#!/usr/bin/env python3
"""
quick PID optimization launcher
start bayesian_opt_node and fast_pid_experiment_client at the same time
"""

import subprocess
import time
import signal
import sys
import os
import psutil

# 全局进程列表
processes = []
shutdown_requested = False

def cleanup_processes():
    """clean up all processes"""
    global processes, shutdown_requested
    
    if shutdown_requested:
        return
        
    shutdown_requested = True
    print('\n🧹 cleaning up all processes...')
    
    # terminate processes gracefully
    for process in processes:
        if process and process.poll() is None:
            try:
                print(f"🛑 terminating process PID: {process.pid}")
                process.terminate()
            except Exception as e:
                print(f"⚠️  error terminating process: {e}")
    
    # wait for processes to finish
    time.sleep(2)
    
    # force kill remaining processes
    for process in processes:
        if process and process.poll() is None:
            try:
                print(f"💀 force killing process PID: {process.pid}")
                process.kill()
            except Exception as e:
                print(f"⚠️  error killing process: {e}")
    
    # kill related Python processes (backup cleanup)
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if proc.info['name'] == 'python3' and proc.info['cmdline']:
                    cmdline = ' '.join(proc.info['cmdline'])
                    if ('bayesian_opt_node' in cmdline or 
                        'fast_pid_experiment_client' in cmdline):
                        print(f"🗑️  cleaning up residual processes: {proc.info['pid']}")
                        proc.terminate()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    except Exception as e:
        print(f"⚠️  error cleaning up residual processes: {e}")
    
    print("✅ processes cleaned up")

def signal_handler(signum, frame):
    """signal handler"""
    print(f'\n🛑 received signal {signum}, exiting safely...')
    cleanup_processes()
    sys.exit(0)

def main():
    global processes
    
    print("="*70)
    print("⚡ fast Crazyflie PID optimization system launcher")
    print("="*70)
    print("🚀 this script will start:")
    print("  1. bayes optimization node (15 iterations)")
    print("  2. fast experiment client (15 seconds experiment)")
    print("")
    print("⚠️  please ensure:")
    print("  • Crazyflie is connected and ready to fly")
    print("  • PhaseSpace system is running")
    print("  • flight area is safe")
    print("")
    print("⏱️  expected optimization time: 15-25 minutes")
    print("🚨 press Ctrl+C to safely stop all processes at any time")
    print("="*70)
    
    # register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # wait for user confirmation
    try:
        input("press Enter to start fast optimization, or Ctrl+C to cancel...")
    except KeyboardInterrupt:
        print("\n🛑 user cancelled")
        sys.exit(0)
    
    try:
        # get script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        print("\n🚀 starting bayes optimization node...")
        bayes_process = subprocess.Popen([
            'python3', 
            os.path.join(script_dir, 'scripts/bayesian_opt_node.py'),
            '--ros-args', 
            '--params-file', 
            os.path.join(script_dir, 'config/fast_pid_optimization_config.yaml')
        ], 
        # remove output redirection, show output directly in terminal
        preexec_fn=os.setsid  # create new process group
        )
        processes.append(bayes_process)
        
        # wait for optimization node to start
        print("⏳ waiting for optimization node to start...")
        time.sleep(3)
        
        # check if bayes node started successfully (simplified check)
        if bayes_process.poll() is not None:
            print(f"❌ bayes optimization node failed to start, exit code: {bayes_process.returncode}")
            return
        
        print("⚡ starting fast experiment client...")
        client_process = subprocess.Popen([
            'python3',
            os.path.join(script_dir, 'scripts/fast_pid_experiment_client.py')
        ],
        # remove output redirection, show output directly in terminal
        preexec_fn=os.setsid  # create new process group
        )
        processes.append(client_process)
        
        print("\n✅ all processes started!")
        print("📊 monitoring optimization progress...")
        print("🚨 press Ctrl+C to safely stop optimization")
        print("="*50)
        
        # monitor process status
        try:
            while True:
                # check if processes are still running
                bayes_running = bayes_process.poll() is None
                client_running = client_process.poll() is None
                
                if not bayes_running and not client_running:
                    print("\n✅ all processes finished normally")
                    break
                elif not bayes_running:
                    print("\n⚠️  bayes optimization node exited")
                    if client_running:
                        print("🛑 stopping experiment client...")
                        client_process.terminate()
                    break
                elif not client_running:
                    print("\n⚠️  experiment client exited")
                    if bayes_running:
                        print("🛑 stopping bayes optimization node...")
                        bayes_process.terminate()
                    break
                
                time.sleep(1)  # check every second
                
        except KeyboardInterrupt:
            print("\n🛑 user interrupted optimization")
            
    except Exception as e:
        print(f"❌ failed to start: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        cleanup_processes()
        print("✅ fast PID optimization exited safely!")

if __name__ == "__main__":
    main() 