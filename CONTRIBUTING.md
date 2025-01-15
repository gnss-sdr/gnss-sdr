# Contributing to GNSS-SDR

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2011-2021 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

:+1::tada: Thanks for taking the time to contribute! :tada::+1:

Third-party contributions are essential for keeping GNSS-SDR continuously
improving. We simply cannot access the huge number of platforms and myriad
configurations for running GNSS-SDR. We want to keep it as easy as possible to
contribute changes that get things working in your environment. There are a few
guidelines that we need contributors to follow so that we can have a chance of
keeping on top of things.

The following is a set of guidelines for contributing to GNSS-SDR, which is
hosted in the [GNSS-SDR Organization](https://github.com/gnss-sdr) on GitHub.
These are just guidelines, not rules. Use your best judgment, and feel free to
propose changes to this document in a
[pull request](#how-to-submit-a-pull-request).

## Code of Conduct

This project adheres to the Contributor Covenant
[code of conduct](CODE_OF_CONDUCT.md). By participating, you are expected to
uphold this code. Please report unacceptable behavior.

## Reporting an issue

Have you found a bug in the code which is not in the
[list of known bugs](https://github.com/gnss-sdr/gnss-sdr/issues)? Do you have a
suggestion for improvement? Then by all means please
[submit a new issue](https://github.com/gnss-sdr/gnss-sdr/issues/new), and do
not hesitate to comment on existing
[open issues](https://github.com/gnss-sdr/gnss-sdr/issues).

When filling a new issue, please remember to:

- **Use a clear and descriptive title** for the issue to identify the problem.

- **Describe the exact steps which reproduce the problem** in as many details as
  possible. For example, start by describing your computing platform (Operating
  System and version, how did you installed GNSS-SDR and its dependencies, what
  file or front-end are you using as a signal source, etc.). You can also
  include the configuration file you are using, or a dump of the terminal output
  you are getting. The more information you provide, the more chances to get
  useful answers.

- **Please be patient**. This organization is run on a volunteer basis, so it
  can take some time to the Developer Team to reach your issue. They will do
  their best to fix it as soon as possible.

- If you opened an issue that is now solved, it is a good practice to **close
  it**.

The list of [open issues](https://github.com/gnss-sdr/gnss-sdr/issues) can be a
good starting point and a source of ideas if you are looking to contribute to
the source code.

## Contributing to the source code

### Preliminaries

1.  If you still have not done so,
    [create your personal account on GitHub](https://github.com/join).

2.  [Fork GNSS-SDR from GitHub](https://github.com/gnss-sdr/gnss-sdr/fork). This
    will copy the whole gnss-sdr repository to your personal account.

3.  Then, go to your favorite working folder in your computer and clone your
    forked repository by typing (replacing `YOUR_USERNAME` by the actual
    username of your GitHub account):

              $ git clone https://github.com/YOUR_USERNAME/gnss-sdr

4.  Your forked repository https://github.com/YOUR_USERNAME/gnss-sdr will
    receive the default name of `origin`. You can also add the original gnss-sdr
    repository, which is usually referred to as `upstream`:

              $ cd gnss-sdr
              $ git remote add upstream https://github.com/gnss-sdr/gnss-sdr.git

To verify the new upstream repository you have specified for your fork, type
`git remote -v`. You should see the URL for your fork as `origin`, and the URL
for the original repository as `upstream`:

```
$ git remote -v
origin    https://github.com/YOUR_USERNAME/gnss-sdr.git (fetch)
origin    https://github.com/YOUR_USERNAME/gnss-sdr.git (push)
upstream  https://github.com/gnss-sdr/gnss-sdr.git (fetch)
upstream  https://github.com/gnss-sdr/gnss-sdr.git (push)
```

### Start working on your contribution

Checkout the `next` branch of the git repository in order to get synchronized
with the latest development code:

```
$ git checkout next
$ git pull upstream next
```

When start working in a new improvement, please **always** branch off from
`next`. Open a new branch and start working on it:

```
$ git checkout -b my_feature
```

Now you can do changes, add files, do commits (please take a look at
[how to write good commit messages](https://chris.beams.io/posts/git-commit/),
and do not forget to
[sign your commits](https://gnss-sdr.org/docs/tutorials/using-git/#sign-your-commits)
if you plan to submit your code to the upstream repository in a pull-request,
see below) and push them to your repository:

```
$ git push origin my_feature
```

If there have been new pushes to the `next` branch of the `upstream` repository
since the last time you pulled from it, you might want to put your commits on
top of them (this is mandatory for pull requests):

```
$ git pull --rebase upstream next
```

### How to submit a pull request

Any code contributions going into GNSS-SDR will become part of a GPL-licensed,
open source repository. It is therefore imperative that code submissions belong
to the authors, and that submitters have the authority to merge that code into
the public GNSS-SDR codebase.

For that purpose, we use the
[Developer's Certificate of Origin](https://github.com/gnss-sdr/gnss-sdr/blob/next/.github/DCO.txt).
It is the same document used by other projects. Signing the DCO states that
there are no legal reasons to not merge your code.

To sign the DCO, suffix your git commits with a `Signed-off-by:` line. When
using the command line, you can use `git commit -s` to automatically add this
line. If there were multiple authors of the code, or other types of
stakeholders, make sure that all are listed, each with a separate
`Signed-off-by:` line. See
[how to sign commits](https://gnss-sdr.org/docs/tutorials/using-git/#sign-your-commits)
for details on how to tell Git to sign commits by default.

Before submitting your code, please be sure to
[apply clang-format](https://gnss-sdr.org/coding-style/#use-tools-for-automated-code-formatting).

When the contribution is ready, you can
[submit a pull request](https://github.com/gnss-sdr/gnss-sdr/compare/). Head to
your GitHub repository, switch to your `my_feature` branch, and click the
_**Pull Request**_ button, which will do all the work for you. Code comparison
must be always to the `next` branch.

Once a pull request is sent, the Developer Team can review the set of changes,
discuss potential modifications, and even push follow-up commits if necessary.

Some things that will increase the chance that your pull request is accepted:

- Your commits must be signed.
- Avoid platform-dependent code. If your code require external dependencies,
  they must be available as packages in
  [Debian OldStable](https://wiki.debian.org/DebianOldStable).
- Write tests.
- Follow our [coding style guide](https://gnss-sdr.org/coding-style/).
  Specifically, please make sure that you have applied
  [clang-format](https://clang.llvm.org/docs/ClangFormat.html) and
  [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) to your changes before
  you do your pull request. Please check the
  [final recommendations](https://gnss-sdr.org/coding-style/#final-recommendations)
  for guidelines on how to apply those tools.
- Write a descriptive and detailed summary. Please consider that reviewing pull
  requests is hard, so include as much information as possible to make your pull
  request's intent clear.

For more details about Git usage, please check out
[our tutorial](https://gnss-sdr.org/docs/tutorials/using-git/).

## Contributing to the website

The content of https://gnss-sdr.org lives in a GitHub repository at
https://github.com/gnss-sdr/geniuss-place

You can fork that repository, reproduce the entire website on your computer
using [Jekyll](https://jekyllrb.com/), do changes and submit pull requests, just
as explained above. For more details, please check out
[how to contribute](https://gnss-sdr.org/contribute/).

Last but not the least, you can leave your comments on the website.

---

![GeNiuSS
contributes](https://gnss-sdr.org/assets/images/geniuss-contribute.png)

Thanks for your contribution to GNSS-SDR!
