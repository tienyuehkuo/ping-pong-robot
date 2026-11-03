# Project collaboration

- Joseph requires completed code changes to be presented as GitHub pull requests,
  not only local branches or commits. Publish the branch and provide the PR link
  as the deliverable. Do not merge without authorization.
- Check existing PR and default-branch state before choosing a base. If a change
  depends on an unmerged PR, use an explicitly documented stacked PR rather than
  silently including the prerequisite changes in the review.
- Verify relevant C++ tests before publishing. See README.md for CMake and direct
  compiler commands. Keep gravity-only reference tests when adding aerodynamics.
