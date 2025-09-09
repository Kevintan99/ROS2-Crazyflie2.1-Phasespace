#!/usr/bin/env python3
"""
ros2_bayes_opt_node.py

A ROS 2 (rclpy) node that runs Bayesian Optimization and coordinates
with a real-world experiment via topics. It supports an arbitrary
number of input parameters (R^n search space), publishes suggested
parameter vectors to test, then *waits* to receive the experiment's
measured objective value and a separate "allow step" flag before
continuing to the next iteration.

Key ideas
---------
- Suggestion out (to your experiment):
    Topic:   /bayes_opt/params_to_test  (std_msgs/Float64MultiArray)
    Payload: ordered vector matching `param_names`

- Experiment -> Node feedback (two topics):
    Topic:   /bayes_opt/experiment_result (std_msgs/Float64)
             The *measured* objective (higher is better by default).

    Topic:   /bayes_opt/allow_step       (std_msgs/Bool)
             Set True when it's OK for the optimizer to register the
             last result and compute the next suggestion.

- Progress/status out:
    Topic:   /bayes_opt/status       (std_msgs/String)
    Topic:   /bayes_opt/best_params  (std_msgs/Float64MultiArray)
    Topic:   /bayes_opt/best_value   (std_msgs/Float64)

Configuration (ROS 2 parameters)
--------------------------------
- param_names        : list[str]    (e.g., ["x1","x2"])  
- lower_bounds       : list[float]  (same length as param_names)
- upper_bounds       : list[float]  (same length as param_names)
- n_iter             : int          (total Bayesian iterations)
- init_points        : int          (random initial points, default 0)
- acquisition_type   : str          ("ucb", "ei", or "poi")
- kappa              : float        (for UCB)
- xi                 : float        (for EI/POI)
- random_state       : int          (for reproducibility)
- objective_mode     : str          ("max" or "min"; default "max")
- timeout_sec        : float        (optional; wait limit for results)
- topics.params_out  : str          (default "/bayes_opt/params_to_test")
- topics.result_in   : str          (default "/bayes_opt/experiment_result")
- topics.allow_in    : str          (default "/bayes_opt/allow_step")
- topics.status      : str          (default "/bayes_opt/status")
- topics.best_params : str          (default "/bayes_opt/best_params")
- topics.best_value  : str          (default "/bayes_opt/best_value")

Dependencies
------------
- ROS 2 (rclpy)
- Python package `bayesian-optimization` (pip install bayesian-optimization)

Notes
-----
- The optimizer *blocks* inside its objective function until the node
  receives BOTH a numeric result and allow_step=True. This lets you
  gate the optimization using your own experiment workflow.
- If `objective_mode` == "min", the node negates results internally
  so that BayesianOptimization (which maximizes) can still be used.
- For more robust bookkeeping, you may add an iteration ID message
  and include it with your experiment result, but this minimal node
  keeps the interface simple.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import (
    Bool,
    Float64,
    Float64MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
    String,
)

from bayes_opt import BayesianOptimization
from bayes_opt import acquisition


class BayesOptNode(Node):
    def __init__(self) -> None:
        super().__init__("bayes_opt_node")

        # ----------------------- Parameters -----------------------
        # Search space definition
        self.declare_parameter("param_names", ["x"])  # type: ignore[arg-type]
        self.declare_parameter("lower_bounds", [-2.0])
        self.declare_parameter("upper_bounds", [10.0])

        # Optimization control
        self.declare_parameter("n_iter", 10)
        self.declare_parameter("init_points", 0)
        self.declare_parameter("acquisition_type", "ucb")  # ucb | ei | poi
        self.declare_parameter("kappa", 2.576)  # UCB exploration factor
        self.declare_parameter("xi", 0.0)       # EI/POI exploration parameter
        self.declare_parameter("random_state", 12345)
        self.declare_parameter("objective_mode", "max")   # max | min
        self.declare_parameter("timeout_sec", 0.0)         # 0.0 => no timeout

        # Topics
        self.declare_parameter("topics.params_out", "/bayes_opt/params_to_test")
        self.declare_parameter("topics.result_in", "/bayes_opt/experiment_result")
        self.declare_parameter("topics.allow_in", "/bayes_opt/allow_step")
        self.declare_parameter("topics.status", "/bayes_opt/status")
        self.declare_parameter("topics.best_params", "/bayes_opt/best_params")
        self.declare_parameter("topics.best_value", "/bayes_opt/best_value")

        # Read parameters
        self.param_names: List[str] = (
            self.get_parameter("param_names").get_parameter_value().string_array_value
        )
        # rclpy returns tuples for arrays; cast to list
        self.param_names = list(self.param_names)

        lower = list(self.get_parameter("lower_bounds").get_parameter_value().double_array_value)
        upper = list(self.get_parameter("upper_bounds").get_parameter_value().double_array_value)
        assert len(lower) == len(upper) == len(self.param_names), (
            "lower_bounds, upper_bounds, and param_names must have the same length"
        )
        self.lower_bounds = lower
        self.upper_bounds = upper

        self.n_iter = int(self.get_parameter("n_iter").value)
        self.init_points = int(self.get_parameter("init_points").value)
        self.acquisition_type = str(self.get_parameter("acquisition_type").value).lower()
        self.kappa = float(self.get_parameter("kappa").value)
        self.xi = float(self.get_parameter("xi").value)
        self.random_state = int(self.get_parameter("random_state").value)
        self.objective_mode = str(self.get_parameter("objective_mode").value).lower()
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        if self.timeout_sec <= 0.0:
            self.timeout_sec = None  # type: ignore[assignment]

        # Topic names
        self.topic_params_out = str(self.get_parameter("topics.params_out").value)
        self.topic_result_in = str(self.get_parameter("topics.result_in").value)
        self.topic_allow_in = str(self.get_parameter("topics.allow_in").value)
        self.topic_status = str(self.get_parameter("topics.status").value)
        self.topic_best_params = str(self.get_parameter("topics.best_params").value)
        self.topic_best_value = str(self.get_parameter("topics.best_value").value)

        # --------------------- Pub/Sub wiring ---------------------
        self.pub_params = self.create_publisher(Float64MultiArray, self.topic_params_out, 10)
        self.pub_status = self.create_publisher(String, self.topic_status, 10)
        self.pub_best_params = self.create_publisher(Float64MultiArray, self.topic_best_params, 10)
        self.pub_best_value = self.create_publisher(Float64, self.topic_best_value, 10)

        self.sub_result = self.create_subscription(
            Float64, self.topic_result_in, self._on_result, 10
        )
        self.sub_allow = self.create_subscription(
            Bool, self.topic_allow_in, self._on_allow, 10
        )

        # ------------------- Coordination state -------------------
        self._lock = threading.Lock()
        self._result_event = threading.Event()
        self._allow_event = threading.Event()
        self._last_result_value: float | None = None
        self._allow_flag: bool = False
        self._iteration = 0
        self._stop_flag = False

        # Start the optimization after a delay to ensure subscribers are connected
        self._started = False
        self._startup_delay = 3.0  # Wait 3 seconds for connections
        self._starter_timer = self.create_timer(self._startup_delay, self._start_once)

        # Log basic configuration
        self.get_logger().info(
            f"Starting Bayesian Optimization with {len(self.param_names)}D space, n_iter={self.n_iter}, init_points={self.init_points}"
        )
        self.get_logger().info(
            f"Params: {self.param_names}\nLower: {self.lower_bounds}\nUpper: {self.upper_bounds}"
        )

    # ---------------------- Subscriber callbacks ----------------------
    def _on_result(self, msg: Float64) -> None:
        with self._lock:
            self._last_result_value = float(msg.data)
            self._result_event.set()
        self.get_logger().info(f"Received experiment result: {msg.data}")

    def _on_allow(self, msg: Bool) -> None:
        with self._lock:
            self._allow_flag = bool(msg.data)
            if self._allow_flag:
                self._allow_event.set()
            else:
                # Reset if a False arrives
                self._allow_event.clear()
        self.get_logger().info(f"Received allow_step={self._allow_flag}")

    # -------------------- Optimizer start/stop hooks --------------------
    def _start_once(self) -> None:
        if self._started:
            return
        
        # Check if we have subscribers before starting
        result_subscribers = self.count_subscribers(self.topic_result_in)
        allow_subscribers = self.count_subscribers(self.topic_allow_in)
        
        if result_subscribers == 0 or allow_subscribers == 0:
            self.get_logger().warn(
                f"Waiting for experiment client to connect... "
                f"(result_subscribers: {result_subscribers}, allow_subscribers: {allow_subscribers})"
            )
            # Try again in 2 seconds
            self._starter_timer = self.create_timer(2.0, self._start_once)
            return
        
        self.get_logger().info(
            f"Experiment client connected! "
            f"(result_subscribers: {result_subscribers}, allow_subscribers: {allow_subscribers})"
        )
        self.get_logger().info("Waiting additional 2 seconds for ROS2 connections to fully establish...")
        
        self._started = True
        self._starter_timer.cancel()
        
        # Wait a bit more for ROS2 pub/sub connections to fully establish
        def delayed_start():
            time.sleep(2.0)  # Additional delay for connection stability
            self.get_logger().info("Starting optimization now!")
            self._run_optimization()
        
        threading.Thread(target=delayed_start, daemon=True).start()

    def destroy_node(self) -> bool:
        # Signal any waiting objective to unblock cleanly
        self._stop_flag = True
        self._result_event.set()
        self._allow_event.set()
        return super().destroy_node()

    # ---------------------- Objective + orchestration ----------------------
    def _publish_params(self, vec: List[float]) -> None:
        arr = Float64MultiArray()
        arr.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="params", size=len(vec), stride=len(vec))],
            data_offset=0,
        )
        arr.data = list(map(float, vec))
        self.pub_params.publish(arr)
        self.get_logger().info(f"Published params_to_test: {vec}")

    def _status(self, text: str) -> None:
        self.pub_status.publish(String(data=text))
        self.get_logger().info(text)

    def _objective(self, **kwargs: float) -> float:
        """Blocking objective for bayes_opt: publishes params and waits for result+allow."""
        # Ordered vector matching self.param_names
        vec = [float(kwargs[name]) for name in self.param_names]

        # Reset gating state BEFORE publishing params
        with self._lock:
            self._last_result_value = None
            self._allow_flag = False  # Reset allow flag for this iteration
            self._result_event.clear()
            self._allow_event.clear()

        # Publish suggestion for the external experiment
        self._publish_params(vec)

        start_time = time.time()
        # Wait until we have BOTH a result and allow=True
        while True:
            if self._stop_flag:
                return 0.0

            # wait on whichever event might arrive first, then check both
            # Use a short timeout to allow periodic stop/timeout checks
            self._result_event.wait(timeout=0.05)
            self._allow_event.wait(timeout=0.05)

            with self._lock:
                have_result = self._last_result_value is not None
                allowed = self._allow_flag
                value = self._last_result_value if have_result else None

            if have_result and allowed:
                break

            # Enforce optional overall timeout
            if self.timeout_sec is not None and (time.time() - start_time) > self.timeout_sec:
                self._status("Timeout waiting for result/allow; returning worst-case placeholder.")
                # For safety, return a very poor value to discourage re-sampling
                return -1e12 if self.objective_mode == "max" else 1e12

        # We have a result and permission to step
        assert value is not None
        self._iteration += 1

        # Report status (use original sign for human readability)
        human_value = value
        self._status(f"Iter {self._iteration}: received objective={human_value}")

        # Transform for maximization if needed
        if self.objective_mode == "min":
            return -float(value)
        return float(value)

    def _run_optimization(self) -> None:
        # Build pbounds dict
        pbounds: Dict[str, tuple[float, float]] = {
            name: (float(lo), float(hi))
            for name, lo, hi in zip(self.param_names, self.lower_bounds, self.upper_bounds)
        }

        # Choose acquisition function
        if self.acquisition_type == "ucb":
            acq = acquisition.UpperConfidenceBound(kappa=self.kappa)
        elif self.acquisition_type == "ei":
            acq = acquisition.ExpectedImprovement(xi=self.xi)
        elif self.acquisition_type == "poi":
            acq = acquisition.ProbabilityOfImprovement(xi=self.xi)
        else:
            self.get_logger().warn(
                f"Unknown acquisition_type '{self.acquisition_type}', defaulting to UCB"
            )
            acq = acquisition.UpperConfidenceBound(kappa=self.kappa)

        optimizer = BayesianOptimization(
            f=self._objective,
            pbounds=pbounds,
            acquisition_function=acq,
            random_state=self.random_state,
            verbose=2,
        )

        self._status("Bayesian optimization started.")
        try:
            optimizer.maximize(
                init_points=self.init_points,
                n_iter=self.n_iter,
            )
        except Exception as e:
            self._status(f"Optimization aborted due to exception: {e}")
        finally:
            # Publish best-so-far in human-readable sign
            if optimizer.max is not None:
                best_params_dict = optimizer.max.get("params", {})
                best_params_vec = [float(best_params_dict[name]) for name in self.param_names]
                best_value_internal = float(optimizer.max.get("target", float("nan")))
                best_value_human = (
                    -best_value_internal if self.objective_mode == "min" else best_value_internal
                )

                arr = Float64MultiArray()
                arr.layout = MultiArrayLayout(
                    dim=[MultiArrayDimension(label="params", size=len(best_params_vec), stride=len(best_params_vec))],
                    data_offset=0,
                )
                arr.data = best_params_vec
                self.pub_best_params.publish(arr)
                self.pub_best_value.publish(Float64(data=best_value_human))
                self._status(
                    f"Optimization complete. Best value={best_value_human}, params={best_params_vec}"
                )
            else:
                self._status("Optimization finished without a valid best result.")


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BayesOptNode()

    # Spin with a MultiThreadedExecutor so callbacks flow while the optimizer
    # runs in its own thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
