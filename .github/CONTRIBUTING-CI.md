# Continuous integration and repository rules

`ci.yml` runs on pull requests, pushes to `main`, release tags, and manual
requests. Clean Ubuntu 24.04 jobs check formatting, Clippy, tests and doctests,
documentation, the Rust 1.88.0 minimum supported version, and WASM compilation.
The final **CI** job succeeds only if every dependency job succeeds; a skipped,
cancelled, or failed job fails that gate. Repository rules require this stable
check name rather than coupling protection to individual matrix jobs.

Workflows use read-only repository permissions, do not retain checkout
credentials, and pin actions to full commit SHAs. No self-hosted runner or
repository secret is needed for the automatic checks.

## Main branch

`rulesets/main.json` is the configuration applied through GitHub's repository
rulesets API. It requires a pull request, resolved review threads, and a passing
**CI** check from GitHub Actions on an up-to-date branch. Approval is optional,
so authors can merge their own pull requests once those requirements pass.
New commits dismiss stale approvals. Force pushes and deletion are blocked.
No role has a configured bypass, including administrators.

## Release tags

`rulesets/release-tags.json` restricts creation, updates, and deletion of `v*`
tags to the repository's Maintain and Admin roles. RepositoryRole IDs 4 and 5
are GitHub's built-in Maintain and Admin roles, respectively.

These JSON files document settings; GitHub does not apply them automatically.
Administrators must keep the live rulesets and these files synchronized.
