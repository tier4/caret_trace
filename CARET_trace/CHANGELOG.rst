^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package caret_trace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* feat(hooked_trace_point.cpp): add tracepoint to cyclonedds for serialized_publish (`#228 <https://github.com/tier4/caret_trace/issues/228>`_)
  * add tracepoint to cyclonedds for serialized_publish
  * ci(pre-commit): autofix
  * ignore cspell check
  * store message addres in thread local memory
  * ci(pre-commit): autofix
  * Update CARET_trace/src/hooked_trace_points.cpp
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
  * Update CARET_trace/src/hooked_trace_points.cpp
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
* add dispatch subscription for generic_subscirption (`#220 <https://github.com/tier4/caret_trace/issues/220>`_)
* Contributors: ymski

0.4.20 (2023-11-10)
-------------------

0.4.19 (2023-10-30)
-------------------

0.4.18 (2023-10-16)
-------------------
* feat: support iron useful tracepoints (`#150 <https://github.com/tier4/caret_trace/issues/150>`_)
  * added skeleton
  * added function call
  * removed duplicated rclcpp_intra_publish
  * add allowed buffer list
  * impl filter
  * add init_tracepoint
  * pass build
  * added callback_register for iron
  * refactor
  * refactor
  * remove unused object
  * add distribution to caret init
  * ci(pre-commit): autofix
  * add test
  * Update CARET_trace/include/caret_trace/tp.h
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
  * Update CARET_trace/include/caret_trace/tracing_controller.hpp
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
  * Update CARET_trace/include/caret_trace/tracing_controller.hpp
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
  * add filter to ring_buffer_clear
  * fix comments for newly added method
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
* Contributors: ymski

0.4.17 (2023-10-03)
-------------------

0.4.16 (2023-09-21 10:10:58 +0900)
----------------------------------

0.4.15 (2023-09-01)
-------------------

0.4.14 (2023-08-21)
-------------------
* feat: change publish trace point definition (`#136 <https://github.com/tier4/caret_trace/issues/136>`_)
  * change publish tp
  * refactor
  ---------
* Contributors: ymski

0.4.13 (2023-07-20)
-------------------
* feat: ignore rcl_timer_init() from ros2 launch command (`#129 <https://github.com/tier4/caret_trace/issues/129>`_)
  * Ignore rcl_timer_init() from ros2 launch command
  * ci(pre-commit): autofix
  * Change sprintf to snprintf
  * ci(pre-commit): autofix
  * internal review results reflected
  * ci(pre-commit): autofix
  * change using-directives to using-declarations
  * ci(pre-commit): autofix
  * Internal review results reflected
  * ci(pre-commit): autofix
  * Change check method to "python3" only
  * confirmed by exclusion in python3
  * reflecting review comments
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: miyakoshi

0.4.12 (2023-07-10)
-------------------

0.4.11 (2023-06-26)
-------------------

0.4.10 (2023-06-07)
-------------------
* feat: support rmw_take tracepoint (`#113 <https://github.com/tier4/caret_trace/issues/113>`_)
  * remove dispatch_subscription_callback tracepoint
  * feat: support rmw_take
  * add: rmw_take filter
  * remove unused variables
  * change the method of not outputting `dispatch_callback_subscription` from deleting functions to adding filters
  * fix code stype
  * fix: compile error
  * Update CARET_trace/src/tracing_controller.cpp
  remove an unnecessary condition from rmw_subscription filter
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
  ---------
  Co-authored-by: isp-uetsuki <35490433+isp-uetsuki@users.noreply.github.com>
* chore: sync files (`#105 <https://github.com/tier4/caret_trace/issues/105>`_)
  * chore: sync files
  * ci(pre-commit): autofix
  ---------
  Co-authored-by: takam5f2 <takam5f2@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: system-tools-actions-public[bot], ymski

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
* feat: enable tracepoint filtering in DDS layer (`#101 <https://github.com/tier4/caret_trace/issues/101>`_)
  * feat: enable tracepoint filtering in DDS layer
  * rename ros2caret_is_rcl_publish_recorded to trace_filter_is_rcl_publish_recorded
  ---------
* Contributors: isp-uetsuki

0.4.4 (2023-02-10)
------------------

0.4.3 (2023-02-07)
------------------
* refactor: remove unnecessary function call (`#93 <https://github.com/tier4/caret_trace/issues/93>`_)
  * refactor: remove unnecessary function call
  * typo
* Contributors: hsgwa

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
* feat: add warning when /clock is not published. (`#77 <https://github.com/tier4/caret_trace/issues/77>`_)
* Contributors: hsgwa

0.3.3 (2022-11-22)
------------------

0.3.2 (2022-11-14)
------------------
* refactor: add context class (`#62 <https://github.com/tier4/caret_trace/issues/62>`_)
  * refactor: add context class
  * fix: build error
* Contributors: hsgwa

0.3.1 (2022-10-31)
------------------
* feat: add APIs to HashableKeys (`#52 <https://github.com/tier4/caret_trace/issues/52>`_)
  * add: comment for hash function
  * remove redundant comment
  * fixe a bug that equals uses only the first argument
  * extend to 5 arguments
  * add :get api(first, second, ... fifth)
  * add less operator
  * add: size tests
  * add: modify string literal case
  * pre-commit
  * typo
  * fix error in release build
* test: add tests for hashable keys (`#46 <https://github.com/tier4/caret_trace/issues/46>`_)
  * test:add tests for hashable keys
  * add: multi key case
  * remove redundant library
* Contributors: hsgwa

0.3.0 (2022-09-20)
------------------
* feat: updated for Humble version release (`#41 <https://github.com/tier4/caret_trace/issues/41>`_)
  * remove unused hook points
  * update dds-related hook points for humble
  * filter unused tracepoints
  * fix: fastdds symbol
  * fix: duplicated add_callbackgroup tracepoints
  * filter rclcpp_take
  * fix: bug that fails to save after the second trace
  * ci(pre-commit): autofix
  * revert: package.xml
  * ci(pre-commit): autofix
  * revert: revert tp.h and ros_trace_points.cpp for clang-off
  * ci(pre-commit): autofix
  * revert: revert hooked_trace_points.cpp for clang-off
  * chore: add NOLINT to line 88 of ros_trace_points.cpp
  * ci(pre-commit): autofix
  * revert: remove NOLINT to line 88 of ros_trace_points.cpp
  * chore: add NOLINT to suppress cpplint warning
  * chore: add missing NOLINT to suppress cpplint warning
  * chore: add literals namespace explicitly
  Co-authored-by: hsgwa <hasegawa.isp@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* ci: replace ament_lint_common with pre-commit and caret_lint_common (`#32 <https://github.com/tier4/caret_trace/issues/32>`_)
  * ci: use pre-commit instead of ament_lint_common
  * style: apply formatting
  * fix: fixed build error
  * ci: delete isort and black
  * fix: reverted tp.h for readability
  * ci(pre-commit): autofix
  * fix: reverted hooked_trace_points and disable clang-format on lines
  * fix: add clang-format for building caret_trace
  * ci(pre-commit): autofix
  * fix: add clang-format off to ros_trace_points
  * ci(pre-commit): autofix
  * fix: add space between comment and slash
  * ci(pre-commit): autofix
  * fix(github-actions): change referenced repository
  Co-authored-by: Takayuki AKAMINE <takayuki.akamine@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* ci(pre-commit): autoupdate (`#27 <https://github.com/tier4/caret_trace/issues/27>`_)
  * ci(pre-commit): autoupdate
  updates:
  - [github.com/pre-commit/pre-commit-hooks: v4.2.0 → v4.3.0](https://github.com/pre-commit/pre-commit-hooks/compare/v4.2.0...v4.3.0)
  - [github.com/igorshubovych/markdownlint-cli: v0.31.1 → v0.32.0](https://github.com/igorshubovych/markdownlint-cli/compare/v0.31.1...v0.32.0)
  - [github.com/pre-commit/mirrors-prettier: v2.6.2 → v2.7.1](https://github.com/pre-commit/mirrors-prettier/compare/v2.6.2...v2.7.1)
  - [github.com/adrienverge/yamllint: v1.26.3 → v1.27.1](https://github.com/adrienverge/yamllint/compare/v1.26.3...v1.27.1)
  - [github.com/scop/pre-commit-shfmt: v3.4.3-1 → v3.5.1-1](https://github.com/scop/pre-commit-shfmt/compare/v3.4.3-1...v3.5.1-1)
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kenji Miyake, Takayuki AKAMINE, pre-commit-ci[bot]

0.2.3 (2022-07-08)
------------------
* chore: rearranged dependent packages (`#23 <https://github.com/tier4/caret_trace/issues/23>`_)
  * chore: rearranged dependent packages
  * chore: delete Install LTTng
  * chore: delete Install LTTng
  * fix: delete comment out
* Contributors: keita1523

0.2.2 (2022-04-14)
------------------
* chore: adapt github actions (`#16 <https://github.com/tier4/caret_trace/issues/16>`_)
  * chore: add target_link_libraries for lttng-ust
  * chore: add commands to install lttng
  * chore: typing miss
  * chore: add cmake flags to resolve gcov error
  * change directory structure.
  * fixed CMakeLists.txt
  * fixed *md files
  * add LTTng installation.
  * fix: delete comment out in CMakeLists
  Co-authored-by: Takayuki AKAMINE <takayuki.akamine@tier4.jp>
* Contributors: keita1523

0.2.1 (2022-01-17)
------------------

0.1.0 (2021-09-16)
------------------
