[('amoeba-beta2', 'amoeba-beta3'), ('amoeba-beta3', 'amoeba-beta4'), ('amoeba-beta4', 'amoeba-beta5'), ('amoeba-beta5', 'amoeba-beta6'), ('amoeba-beta6', 'HEAD')]

Changelog
=========

amoeba-beta6..HEAD
^^^^^^^^^^^^^^^^^^

9958229 Adding a little bit of documentation for the static_registration.rst
0fb4b14 Disable the changelog for now.
ccc7e88 Working on opts documentation. Adding docs for YAML dump load.
32fce8d Adding changelog that uses gitlog... Its a hack for now but we can make it better. Refer #163
0a3d6b0 Adding yaml dump/load for parameters. Includes test.
6be4d53 HACK blackbox exception handling... #187 This will get fixed most likely when we enable proper scoping of subplasms

amoeba-beta5..amoeba-beta6
^^^^^^^^^^^^^^^^^^^^^^^^^^

f72576c Readme.
a775c31 Adding support for static registration of spores. Refer #186 At preconfigure time for cpp cells, any tendril objects declared with a new static time registration signature are initialized.
3bfccb1 Bump the abi, and clean up a little of the tendril impl.
05144c4 Adding a few more return values, and a boost sequence foreach string name returner.
f66fb3f Cell cleanup, move a bunch out of the header.
f233a51 Minor log tweak.
185d26a BlackBox works!  refer #187.
a415393 Fix for the rename from module to cell.
975a9c7 Working on blackbox, see #187, and args. Refactor modules ~> cells a bit more.
1f0dc67 Woops, silly me.
7b56951 Enable additional test for black box.
bdf7cf3 close #100
c3e68e1 Fix build 125.
595f26e Blackbox still needs a bit of work.
c1d0362 Removing functor target.
4754395 fix test that was no-opped while developing.
fb843e9 Horrible hacks to get pretty formatting of exception diagnostics working on 1.40...   but does work.
54fe1a5 start of cmake docs
bb63a65 make possible to override projects on the cmake line
7425aca fix link errors provoked by having boost before FindDeps
3a4ece7 Attempt to diagnose strange problems on lucid.  No idea if this will work.
66743fe Merge branch 'master' of github.com:plasmodic/ecto
784a27d what does this do on lucid?
dd5694f valgrind docs tweak
80000a9 Merge branch 'master' of github.com:plasmodic/ecto
fa8fc11 Migrating kitchen docs... still a bit messy.
c456f1f Fix up the python path stuff.
4542665 tweak fastidious flags for lucid (-Wno-strict-aliasing), as we're getting an error that clearly has nothing todo with the underlying code... there are no pointers being dereferenced anywhere near the location that the compiler is reporting.
f560a09 Doc errata gets built if docs are built.
5bf0f33 Removing a duplicate theme.
0ded6f2 Merge branch 'master' of github.com:plasmodic/ecto
b81409b Remove duplication.
10a41a7 Full overhaul of exception handling. 1.  Use boost::exception 2.  Wrap different exception types on the python side so that they're catchable
16cce73 Enable post install tests.
45216d0 Moving our sphinx doc directives to our python lib.
aff2134 Getting there with standalone docs.
4502358 Adding xml output to gtest.
148c100 These tests are not very fun.
41c2940 Another pass, why am i so ***
e4c67aa Woops try to fix it again.
3b32138 Fix test on slow build vms.
a1d1b25 Fixing simlink to __init__.py
734803a Creating www-deploy target, a bit of doc editing.
5afb9b1 Comment out the graphviz css crud.
5ac7113 Working on docs in standalone...
5d2fa73 Moving some stuff around in CMake so i can think.
cdcf3fc Adding option for saving a dotfile of the graph.

amoeba-beta4..amoeba-beta5
^^^^^^^^^^^^^^^^^^^^^^^^^^

6e1b08a Dependencies,control file, Just install to a normal place.
c7695a9 Adding a multprocess test.
55ea9fb Adding python original object to the streabuf.
c299e54 Working on #191, file like object support.  Tests included.
c8e780b Refer to #188.  Add implementation, test, docs for the Dealer cell. Closes #188.
e8efd5d Test passes now.
5fa3026 Failing test.
4f6ef28 remove verbosity.
c5a0c09 Little clean up options for ipython version and adding logging option.
37c7f86 Remove cruft from tendrils.hpp.
2b6dd12 Fix up of the python cell, a bit. This needs major work if its to be supported.
7fd4683 rosbuild_lite tweaks.
9c77fcc Doc option tweak, and CMAKE_BUILD_TYPE correction for standalone.
0a5902c Merge branch 'master' of github.com:plasmodic/ecto
690ed90 Working on debian of ecto.
2d2b524 Rosbuild lite messages had no type, so showed up as warnings.
e2eac41 Revert force.
5120ef3 Force the doc target.
d7e4149 Adding cell options.

amoeba-beta3..amoeba-beta4
^^^^^^^^^^^^^^^^^^^^^^^^^^

ab7acf3 tweaks for tests running on very, very slow buildbot vms
6a55e9e this test is failing on the buildbots, want to see what the actual numbers are on a heavily loaded VM
773cad9 Disable doc install.
b5fc2a6 Adding a virtualenv option.
0477e10 This is a major hack... FIXME disable test, refer  #182
4146c99 handle when the error is not fatal in rospack. e.g. [rospack] Failed to change UID; cache permissions may need to be adjusted manually. setuid(): Operation not permitted
3958187 Making rosbuild lite work without sourcing a setup.sh.
c10221b More #182
a45b284 Attempt to address issue #182.
9588f26 Adding install for docs.
e7e30c9 Better permissions on the deploy.
e8e3d85 Making docs a bit more flexible, adding deploy cache variables.
bc7a003 Doc tweak.
4dfa6d3 Install targets.
f5b2722 Minor virtualenv fixes.
65c6b46 Adding virtual env stuffs.
64d8af7 Quiet on the cmake flags front. Fix fastidious, this wasn't getting picked up... Also, working on install targets for ecto modules.
490dad1 tabs to spaces...
9ca6b30 Reverse order of rsync in docs, so that kitchen can override them.
fbacb77 Merge branch 'master' of git://github.com/plasmodic/ecto
344bd64 provoking the buildbots
293c989 provoking the buildbots
f63e6d6 provoking the buildbots
83b5905 provoking the buildbots
fbe31e1 provoking the buildbots
39901c4 provoking the buildbots
368595c provoking the buildbots
12f6745 provoking the buildbots
6457588 provoking the buildbots
e3c2113 provoking the buildbots
e229b46 Merge branch 'master' of github.com:plasmodic/ecto
c2f7290 provoking the buildbots
afa306c Readme touch.
e6d0c69 Removing the ecto_kitchen docs from ecto, put the in ecto_kitchen.
df3d559 doxygen complains about this
c0dcefa provoking the buildbots
ba2d0f8 provoking the buildbots
5756225 provoking the buildbots
582ca27 provoking the buildbots
0c1edd1 provoking the buildbots
b658ee0 provoking the buildbots
1a28085 provoking the buildbots
0ef52d9 provoking the buildbots
888f30c provoking the buildbots
3402bec Merge branch 'master' of github.com:plasmodic/ecto
72d25e5 provoking the buildbots
77facb5 Adding space.
0a28bfa Put in if check to the python path to stop errors when ros does not exist.
ad55b58 Merge branches 'master' and 'amoeba-beta0'
19c354a Adding a bit to the python options.
b15c372 just provoking the buildbot
dadd9e9 tweak
3d3872c use long and short git tags...   not sure how this got lost

amoeba-beta2..amoeba-beta3
^^^^^^^^^^^^^^^^^^^^^^^^^^

23f1983 docs on how to get tagged releases
13d9675 tweak

