# Contributing to GHand-ROS2

Thank you for your interest in improving GHand-ROS2. This document explains how to report issues, propose changes, and submit pull requests.

## Reporting Issues

Before opening a new issue, please search existing issues to avoid duplicates. When reporting a bug, include:

- ROS 2 distribution (e.g., Humble, Jazzy)
- Operating system version
- Steps to reproduce the problem
- Expected behavior and actual behavior
- Relevant logs or error messages
- Whether you are running Gazebo or real hardware

## Proposing Changes

For large changes, please open an issue first to discuss the design. This helps avoid wasted effort and ensures the change aligns with project goals.

## Pull Request Process

1. Fork the repository and create a feature branch.
2. Make focused, minimal changes that address a single concern.
3. Follow the existing code style:
   - C++: Google C++ Style Guide, formatted with the provided `.clang-format`
   - Python: Google Python Style Guide
4. Ensure `colcon build` succeeds for Gazebo development without new warnings.
5. Update relevant documentation (README, package README, CHANGELOG) if the change is user-visible.
6. Submit the pull request with a clear description and link to any related issue.

## Important Policy: No Test Files in the Public Repository

Per company policy, **test files and test code must not be committed to this public repository**. Internal functional and hardware-in-the-loop testing is maintained by a separate internal test team. Pull requests that add `test/` directories, `*_test.cpp` files, or testing-only dependencies will not be accepted.

## Local Development Tooling

The `.claude/` and `openspec/` directories contain Claude Code skills, OpenSpec
artifacts, and internal planning notes used by the maintainers. They are
development-only and should not be committed to public release branches.

## Code Review

Maintainers will review pull requests as time permits. Please be responsive to feedback and keep discussions constructive.

## Code of Conduct

Please read and follow our [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
