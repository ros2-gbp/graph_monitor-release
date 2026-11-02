^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_stats_shim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.3.2 (2026-11-02)
------------------
* fix(rmw_stats_shim): populate TopicStatistic min and max (`#79 <https://github.com/ros-tooling/graph-monitor/issues/79>`_)
  Fixes `#78 <https://github.com/ros-tooling/graph-monitor/issues/78>`_.
  `EndpointStatistics::periodMsg()` and `ageMsg()` set only `mean`, so
  `min` and `max` in every `TopicStatistic` stayed zero.
  `RollingMeanAccumulator` had nothing to fill them from, since it only
  kept a running sum.
  This adds `getRollingMin()` and `getRollingMax()` to
  `RollingMeanAccumulator`.
  They scan the values still in the window, so they cover the same
  `ROS_TOPIC_STATISTICS_WINDOW_SIZE` samples as the mean, and values that
  leave the window drop out of both.
  `periodMsg()` and `ageMsg()` now fill `min` and `max` from them.
  The scan is linear in the window size and runs once per published
  statistic, not per message.
  Tests:
  - `test_rolling_mean_accumulator.cpp` checks min and max on a partly
  filled window and after the window wraps.
  - `EndpointStatistics.MessagesReportWindowMinAndMax` in
  `test_stat_collector.cpp` feeds periods of 10, 20 and 6 ms and ages of
  7, 3 and 15 ms, then checks `mean`, `min` and `max` in both messages.
  Without the change in `stat_collector.cpp` it fails with `min` and `max`
  at 0.
  Built with colcon on Jazzy (`rosgraph_monitor_msgs` and `rmw_stats_shim`
  with `BUILD_TESTING=ON`); `test_rolling_mean_accumulator`,
  `test_stat_collector` and `test_timer` pass, and the pre-commit hooks
  pass on the changed files.
  ---------
* fix(rmw_stats_shim): lock StatCollector endpoint maps (`#76 <https://github.com/ros-tooling/graph-monitor/issues/76>`_)
  Follows up `#75 <https://github.com/ros-tooling/graph-monitor/issues/75>`_.
  `StatCollector` is a process-wide singleton.
  Its maps (`nodes\_`, `stat_publishers\_`, `publishers\_`, `subscriptions\_`)
  were accessed with no locking from three kinds of thread:
  - DDS and executor threads: `onPublish`/`onReceive`, called from the
  wrapped `rmw_publish`/`rmw_take*`.
  - Any thread creating or destroying entities: `add*`/`remove*`.
  - The stats `Timer` thread: `publishStatistics`.
  A rehash during an insert while another thread ran `find` caused a
  SIGSEGV in `StatCollector::onReceive` (`stat_function.h`/`find` frame)
  during test teardown in a downstream CI.
  To solve it, add mutex protection for this state.
  Use separate mutexes for nodes vs endpoints.
  ## Performance
  This adds one uncontended mutex lock per publish/take in every process
  with the shim loaded.
  The downstream project plans to benchmark it.
* fix(rmw_stats_shim): make Timer::stop idempotent (`#75 <https://github.com/ros-tooling/graph-monitor/issues/75>`_)
  ## Problem
  Since `#50 <https://github.com/ros-tooling/graph-monitor/issues/50>`_, `~StatCollector` calls `timer\_->stop()` explicitly. Then
  `~optional<Timer>` runs `~Timer`, which calls `stop()` again. The second
  call does `thread\_.join()` on an empty `std::thread`, which throws
  `std::system_error` (`Invalid argument`) during static destruction.
  As a result, in 0.3.0 every process that loads the shim
  (`RMW_IMPLEMENTATION_WRAPPER=rmw_stats_shim`) aborts on a normal exit:
  ```
  terminate called after throwing an instance of 'std::system_error'
  what():  Invalid argument
  ... Timer::stop          timer.cpp:43
  ... Timer::~Timer        timer.cpp:16
  ... StatCollector::~StatCollector  stat_collector.cpp:309
  ```
  This showed up as 19 failing test executables downstream when we bumped
  to 0.3.0: the tests passed, but every process SIGABRTed at exit.
  ## Fix
  Guard the join with `joinable()`, matching the header's documented
  contract for `stop()` ("No-op if not running").
  ## Tests
  Adds `rmw_stats_shim/test/test_timer.cpp`, the package's first tests:
  stop without start, stop twice, destroy after stop, restart after stop.
  They fail on `main` and pass with this change (built and run on Jazzy).
  ## Not in this PR
  `StatCollector`'s `nodes\_`, `publishers\_` and `subscriptions\_` maps are
  still accessed without a lock from DDS listener threads
  (`onReceive`/`onPublish`), the main thread (endpoint add/remove) and the
  stats timer thread. We've seen a SIGSEGV in `onReceive` from this. I'll
  follow up separately.
* fix(rmw_stats_shim): lock StatCollector endpoint maps (`#76 <https://github.com/ros-tooling/graph-monitor/issues/76>`_)
* fix(rmw_stats_shim): make Timer::stop idempotent (`#75 <https://github.com/ros-tooling/graph-monitor/issues/75>`_)
* Contributors: Emerson Knapp, Rahul Seervi

0.3.0 (2026-10-07)
------------------
* fix: lyrical+rolling tests (`#57 <https://github.com/ros-tooling/graph-monitor/issues/57>`_)
* lint: Move from ament linters to polymath_code_standard autoformatters (`#53 <https://github.com/ros-tooling/graph-monitor/issues/53>`_)
* Link rmw_stats_shim against Threads (`#52 <https://github.com/ros-tooling/graph-monitor/issues/52>`_)
* Fix race condition in StatCollector destructor (`#50 <https://github.com/ros-tooling/graph-monitor/issues/50>`_)
* Use ros_environment to detect distro instead of ament_cmake version (`#39 <https://github.com/ros-tooling/graph-monitor/pull/39>`_)
* Add documentation skeleton for all released packages (`#44 <https://github.com/ros-tooling/graph-monitor/pull/44>`_)
* Add new methods added to graph interface in Rolling (`#48 <https://github.com/ros-tooling/graph-monitor/pull/48>`_)
* Fix race condition in StatCollector destructor (`#50 <https://github.com/ros-tooling/graph-monitor/pull/50>`_)
* Contributors: Emerson Knapp, shrujan, Skyler Medeiros, Tobias Fischer

0.2.3 (2025-10-22)
------------------
* Fix circular dependency (`#38 <https://github.com/ros-tooling/graph-monitor/issues/38>`_)
* Contributors: Emerson Knapp

0.2.2 (2025-10-16)
------------------

0.2.1 (2025-10-15)
------------------

0.1.2 (2025-05-12)
------------------

0.1.1 (2025-04-09)
------------------
* Initial package setup
* Contributors: Emerson Knapp
