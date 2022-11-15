# Contributing to Krakatoa

Thanks for your interest in contributing to Krakatoa! ❤️

This document describes how to set up a development environment and submit your contributions. Please read it carefully
and let us know if it's not up-to-date (even better, submit a PR with your  corrections ;-)).

- [Prerequisites](#prerequisites)
- [Building](#building)
  - [Testing](#testing)
  - [Publishing to Local Conan Cache](#publishing-to-local-conan-cache)
  - [Building Multiple Configurations](#building-multiple-configurations)
- [Pull Requests](#pull-requests)
  - [Pull Request Checklist](#pull-request-checklist)
  - [Step 1: Open Issue](#step-1-open-issue)
  - [Step 2: Design (optional)](#step-2-design-optional)
  - [Step 3: Work your Magic](#step-3-work-your-magic)
  - [Step 4: Commit](#step-4-commit)
  - [Step 5: Pull Request](#step-5-pull-request)
  - [Step 6: Merge](#step-6-merge)

## Prerequisites

You will need Python 3.10 or later and the C++ compiler for your platform installed before you are able to build Krakatoa. The compilers used for each platform are as follows:

* Windows: Visual C++ 14.1 (Visual Studio 2017) or later
* macOS: Clang 10.0 or later
* Linux: GCC 7 or later

You will then need to install Conan. You can do this by running:

```bash
pip install conan conan_package_tools
```

Conan is a C++ package manager that is used to install the 3rd party dependencies.

Krakatoa uses the C++17 standard.

You will need to build the following dependencies to your local Conan cache:

* https://github.com/aws/thinkbox-cm-library
* https://github.com/aws/thinkbox-library

## Building

For Krakatoa follow these steps in the Krakatoa subdirectory. For KrakatoaSR follow these steps in the KrakatoaSR subdirectory. You will need Krakatoa built to your local Conan cache before you can build KrakatoaSR.

From the project root directory run the following commands to install the dependencies and build Krakatoa:

```bash
conan install . --install-folder build
conan build . --build-folder build
```

If you wish to generate a development environment without building the package immediately, you can add `--configure` to the `conan build` command.

If you are using Windows, once run, you can open `build/Krakatoa.sln` or `build/KrakatoaSR.sln` in Visual Studio to use Visual Studio for development and debugging.

### Publishing to Local Conan Cache

If you need to publish build artifacts to your local Conan cache manually, after completing the [building](#building) steps, you can run the following commands to package Krakatoa and publish it to your local Conan cache:

```bash
conan package . --install-folder build --build-folder build --package-folder build/package
conan export-pkg . --package-folder build/package
```

### Building Multiple Configurations

To quickly build all supported configurations on the current platform you can run:

```
python build.py
```

This will build the configurations and publish them to your local conan cache. You can add the `--dry-run` flag to preview the configurations that will be built without building them.

### Pull Requests

#### Pull Request Checklist

- Testing
  - Unit test added (prefer not to modify an existing test, otherwise, it's probably a breaking change)
- Title and Description
  - __Change type__: title prefixed with **fix**, **feat** and module name in parens, which will appear in changelog
  - __Title__: use lower-case and doesn't end with a period
  - __Breaking?__: last paragraph: "BREAKING CHANGE: <describe what changed + link for details>"
  - __Issues__: Indicate issues fixed via: "**Fixes #xxx**" or "**Closes #xxx**"

#### Step 1: Open Issue

If there isn't one already, open an issue describing what you intend to contribute. It's useful to communicate in
advance, because sometimes, someone is already working in this space, so maybe it's worth collaborating with them
instead of duplicating the efforts.

#### Step 2: Design (optional)

In some cases, it is useful to seek for feedback by iterating on a design document. This is useful
when you plan a big change or feature, or you want advice on what would be the best path forward.

Sometimes, the GitHub issue is sufficient for such discussions, and can be sufficient to get
clarity on what you plan to do. Sometimes, a design document would work better, so people can provide
iterative feedback.

In such cases, use the GitHub issue description to collect **requirements** and
**use cases** for your feature.

#### Step 3: Work your Magic

Work your magic. Here are some guidelines:

- Coding style:
  - Code should conform to the style defined in .clang-format
- Every change requires a unit test
- Try to maintain a single feature/bugfix per pull request. It's okay to introduce a little bit of housekeeping
   changes along the way, but try to avoid conflating multiple features. Eventually all these are going to go into a
   single commit, so you can use that to frame your scope.

#### Step 4: Commit

Create a commit with the proposed changes:

- Commit title and message (and PR title and description) must adhere to [conventionalcommits](https://www.conventionalcommits.org).
  - The title must begin with `feat: title`, `fix: title`, `refactor: title` or
    `chore: title`.
  - Title should be lowercase.
  - No period at the end of the title.

- Commit message should describe _motivation_. Think about your code reviewers and what information they need in
  order to understand what you did. If it's a big commit (hopefully not), try to provide some good entry points so
  it will be easier to follow.

- Commit message should indicate which issues are fixed: `fixes #<issue>` or `closes #<issue>`.

- Shout out to collaborators.

- If not obvious (i.e. from unit tests), describe how you verified that your change works.

- If this commit includes breaking changes, they must be listed at the end in the following format (notice how multiple breaking changes should be formatted):

```
BREAKING CHANGE: Description of what broke and how to achieve this behavior now
- **module-name:** Another breaking change
- **module-name:** Yet another breaking change
```

#### Step 5: Pull Request

- Push to a personal GitHub fork.
- Submit a Pull Request on GitHub. A reviewer will later be assigned by the maintainers.
- Please follow the PR checklist written above. We trust our contributors to self-check, and this helps that process!
- Discuss review comments and iterate until you get at least one "Approve". When iterating, push new commits to the
  same branch. Usually all these are going to be squashed when you merge to master. The commit messages should be hints
  for you when you finalize your merge commit message.
- Make sure to update the PR title/description if things change. The PR title/description are going to be used as the
  commit title/message and will appear in the CHANGELOG, so maintain them all the way throughout the process.

#### Step 6: Merge

- Make sure your PR builds successfully
- Once approved and tested, a maintainer will squash-merge to master and will use your PR title/description as the
  commit message.
