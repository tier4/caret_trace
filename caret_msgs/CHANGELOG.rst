^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package caret_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2024-01-31)
------------------
* chore: update package.xml version to 0.4.24 (`#266 <https://github.com/tier4/caret_trace/issues/266>`_)
* Contributors: h-suzuki-isp

0.4.24 (2024-01-05)
-------------------

0.4.23 (2023-12-15)
-------------------
* refactor: upgrade xml version (`#258 <https://github.com/tier4/caret_trace/issues/258>`_)
* Contributors: h-suzuki-isp

0.4.22 (2023-12-12)
-------------------

0.4.21 (2023-11-27)
-------------------
* chore: update maintainer (`#244 <https://github.com/tier4/caret_trace/issues/244>`_)
* Contributors: ymski

0.4.20 (2023-11-10)
-------------------

0.4.19 (2023-10-30)
-------------------

0.4.18 (2023-10-16)
-------------------

0.4.17 (2023-10-03)
-------------------

0.4.16 (2023-09-21 10:10:58 +0900)
----------------------------------

0.4.15 (2023-09-01)
-------------------

0.4.14 (2023-08-21)
-------------------

0.4.13 (2023-07-20)
-------------------

0.4.12 (2023-07-10)
-------------------

0.4.11 (2023-06-26)
-------------------

0.4.10 (2023-06-07)
-------------------

0.4.9 (2023-05-15)
------------------

0.4.8 (2023-04-04 09:47:26 +0900)
---------------------------------

0.4.7 (2023-04-04 09:47:26 +0900)
---------------------------------

0.4.6 (2023-04-04 09:47:26 +0900)
---------------------------------

0.4.5 (2023-03-16)
------------------

0.4.4 (2023-02-10)
------------------

0.4.3 (2023-02-07)
------------------

0.4.2 (2023-01-20)
------------------

0.4.1 (2022-12-26)
------------------

0.4.0 (2022-12-16)
------------------
* feat: add runtime recording feature (`#68 <https://github.com/tier4/caret_trace/issues/68>`_)
  * add: caret_msgs
  * clean CMakeLists
  * docs(keys_set): add document for HashableKeys
  * chore(keys_set): add const to has()
  * chore(keys_set): extend arguments for T4 and T5
  * chore(keys_set): add apis for iterator
  * fix(keys_set): change unordered_set to set
  * add: lttng_session
  * fix: compile warnings
  * chore(tracing_controller): add use_log flag
  * add: recordable_data
  * add: data_container
  * add: data_recorder
  * add: trace node
  * feat(context): data container and trace node
  * test(scenario): add scenario test
  * feat: add runtime recording
  * link lttng libraries to pass build
  * remmove death test
  * typo
  * pass xmllint
  * fix: invalid output of 'failed to load regular expression'
  * fix: starting recording with session fails
  * modify to reduce is_session_running call
  * fix: test_scenario test
  * add: clock
  * add: caret_init tracepoint
  * add: ros2_caret:rcl_timer_init
  * add: caret_init recording
  * fix: RCLInvalidArgument during ros2 doctor --report
  * add doxygen-style comments
  * remove unused using statement
  * add private statement in clock recorder
  * typo
  * add: ros2_caret tracepoints for ros2 initial tracepoints
  * extended up to 6 arguments.
  * add initialization time argument
  * fix: status constant value
  * support: python impl node
  * modify default recording_frequency to 100 hz
  * ci(pre-commit): autofix
  * move DEBUG_OUTPUT to record function
  * apply filtering to runtime recording
  * move current time recording to the beginning of the function.
  * modify to always start a timer when start_message is received.
  * rename is_recording_enabled to is_recording_allowed
  * fix: race condition from PREPARE state to RECORD state.
  * fix: incorrect reader writer lock
  * add: comment
  * modify to record always iniitialization trace points
  * rename start.node_name to start.caret_node_name
  * add reserved message fields
  * ci(pre-commit): autofix
  * modify the order of caret_init tracepoint and state transition
  * clean indent
  * clean indent
  * remove unused function: is_recording_allowed_init
  * rename private variable name: is_end to is_end_iterator
  * add comment
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: hsgwa

0.3.4 (2022-12-13)
------------------

0.3.3 (2022-11-22)
------------------

0.3.2 (2022-11-14)
------------------

0.3.1 (2022-10-31)
------------------

0.3.0 (2022-09-20)
------------------

0.2.3 (2022-07-08)
------------------

0.2.2 (2022-04-14)
------------------

0.2.1 (2022-01-17)
------------------

0.1.0 (2021-09-16)
------------------
