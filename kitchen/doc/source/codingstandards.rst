Coding Standards
================

This is something one doesn't want to overlegislate.

We mainly follow the coding standards of `Sutter and Alexandrescu
<http://www.amazon.com/Coding-Standards-Rules-Guidelines-Practices/dp/0321113586>`_.

A few specific guidelines:

* **NO TABS** in your commits.
* **NO TRAILING WHITESPACE** in the code.

The two above keep noise out of our commit history.  Things we strive for:

* Put the bugfix, the test of the bugfix, and the updated docs in the
  *same commit*, via ``git rebase`` if necessary.  We're not kernel
  ninjas, but we strive to be as good with source control as they
  are.







