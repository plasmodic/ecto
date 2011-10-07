
Changelog
=========
This is a generated changelog file.



amoeba-beta6..amoeba-0
^^^^^^^^^^^^^^^^^^^^^^

============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
`68b1304 <https://github.com/plasmodic/ecto/commit/68b1304>`_ Merge branch 'master' of github.com:plasmodic/ecto
`3f3fb03 <https://github.com/plasmodic/ecto/commit/3f3fb03>`_ A few option docs.
`bf9ebe8 <https://github.com/plasmodic/ecto/commit/bf9ebe8>`_ Enable the fileIO test.
`9239aa7 <https://github.com/plasmodic/ecto/commit/9239aa7>`_ - fix the dealer to have a default empty list
`c9d9795 <https://github.com/plasmodic/ecto/commit/c9d9795>`_ Adding PP fanciness to detect if the module is correctly named based on -Dmy_module_ectomodule_EXPORTS that cmake passes.
`0246b78 <https://github.com/plasmodic/ecto/commit/0246b78>`_ spelling.
`6c1da94 <https://github.com/plasmodic/ecto/commit/6c1da94>`_ Merge branch 'master' of git://github.com/plasmodic/ecto
`a0ab180 <https://github.com/plasmodic/ecto/commit/a0ab180>`_ Rework of static declaration, with a few comile tests.
`10b5b56 <https://github.com/plasmodic/ecto/commit/10b5b56>`_ Adding a few typedefs to enable beter metaprogramming.
`3adb94b <https://github.com/plasmodic/ecto/commit/3adb94b>`_ - fix cases with a bad until
`4474e87 <https://github.com/plasmodic/ecto/commit/4474e87>`_ Fix the raise statement, so that the doc build gives more reasonable errors.
`03ffd66 <https://github.com/plasmodic/ecto/commit/03ffd66>`_ Exception handling HACK in BlackBox. The scoping seems to be high on priority list.
`7e65331 <https://github.com/plasmodic/ecto/commit/7e65331>`_ Incremental improvement on the enumeration of scheduler types.
`53998b8 <https://github.com/plasmodic/ecto/commit/53998b8>`_ Adding dunderscores to member variables, except for _cell in BlackBox interface.
`d025cc7 <https://github.com/plasmodic/ecto/commit/d025cc7>`_ Fixing bad merge for the gtest target.
`5a6ea86 <https://github.com/plasmodic/ecto/commit/5a6ea86>`_ Fixing a type issue in the blackbox, refer #187. Also adding better printing for python object tendrils. This is the inspiration for #194
`2ce54f2 <https://github.com/plasmodic/ecto/commit/2ce54f2>`_ Serialization.
`d7d0e26 <https://github.com/plasmodic/ecto/commit/d7d0e26>`_ Changelog bump.
`7771c6b <https://github.com/plasmodic/ecto/commit/7771c6b>`_ Update changelog generation file.
`b7eed39 <https://github.com/plasmodic/ecto/commit/b7eed39>`_ Adding hard coded changelog.
`9958229 <https://github.com/plasmodic/ecto/commit/9958229>`_ Adding a little bit of documentation for the static_registration.rst
`0fb4b14 <https://github.com/plasmodic/ecto/commit/0fb4b14>`_ Disable the changelog for now.
`ccc7e88 <https://github.com/plasmodic/ecto/commit/ccc7e88>`_ Working on opts documentation. Adding docs for YAML dump load.
`32fce8d <https://github.com/plasmodic/ecto/commit/32fce8d>`_ Adding changelog that uses gitlog... Its a hack for now but we can make it better. Refer #163
`0a3d6b0 <https://github.com/plasmodic/ecto/commit/0a3d6b0>`_ Adding yaml dump/load for parameters. Includes test.
`6be4d53 <https://github.com/plasmodic/ecto/commit/6be4d53>`_ HACK blackbox exception handling... #187 This will get fixed most likely when we enable proper scoping of subplasms
`d72b461 <https://github.com/plasmodic/ecto/commit/d72b461>`_ bing this will get lost again
`d627929 <https://github.com/plasmodic/ecto/commit/d627929>`_ bing this will get lost
============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================


amoeba-beta5..amoeba-beta6
^^^^^^^^^^^^^^^^^^^^^^^^^^

============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
`f72576c <https://github.com/plasmodic/ecto/commit/f72576c>`_ Readme.
`a775c31 <https://github.com/plasmodic/ecto/commit/a775c31>`_ Adding support for static registration of spores. Refer #186 At preconfigure time for cpp cells, any tendril objects declared with a new static time registration signature are initialized.
`3bfccb1 <https://github.com/plasmodic/ecto/commit/3bfccb1>`_ Bump the abi, and clean up a little of the tendril impl.
`05144c4 <https://github.com/plasmodic/ecto/commit/05144c4>`_ Adding a few more return values, and a boost sequence foreach string name returner.
`f66fb3f <https://github.com/plasmodic/ecto/commit/f66fb3f>`_ Cell cleanup, move a bunch out of the header.
`f233a51 <https://github.com/plasmodic/ecto/commit/f233a51>`_ Minor log tweak.
`185d26a <https://github.com/plasmodic/ecto/commit/185d26a>`_ BlackBox works!  refer #187.
`a415393 <https://github.com/plasmodic/ecto/commit/a415393>`_ Fix for the rename from module to cell.
`975a9c7 <https://github.com/plasmodic/ecto/commit/975a9c7>`_ Working on blackbox, see #187, and args. Refactor modules ~> cells a bit more.
`1f0dc67 <https://github.com/plasmodic/ecto/commit/1f0dc67>`_ Woops, silly me.
`7b56951 <https://github.com/plasmodic/ecto/commit/7b56951>`_ Enable additional test for black box.
`bdf7cf3 <https://github.com/plasmodic/ecto/commit/bdf7cf3>`_ close #100
`c3e68e1 <https://github.com/plasmodic/ecto/commit/c3e68e1>`_ Fix build 125.
`595f26e <https://github.com/plasmodic/ecto/commit/595f26e>`_ Blackbox still needs a bit of work.
`c1d0362 <https://github.com/plasmodic/ecto/commit/c1d0362>`_ Removing functor target.
`4754395 <https://github.com/plasmodic/ecto/commit/4754395>`_ fix test that was no-opped while developing.
`fb843e9 <https://github.com/plasmodic/ecto/commit/fb843e9>`_ Horrible hacks to get pretty formatting of exception diagnostics working on 1.40...   but does work.
`54fe1a5 <https://github.com/plasmodic/ecto/commit/54fe1a5>`_ start of cmake docs
`bb63a65 <https://github.com/plasmodic/ecto/commit/bb63a65>`_ make possible to override projects on the cmake line
`7425aca <https://github.com/plasmodic/ecto/commit/7425aca>`_ fix link errors provoked by having boost before FindDeps
`3a4ece7 <https://github.com/plasmodic/ecto/commit/3a4ece7>`_ Attempt to diagnose strange problems on lucid.  No idea if this will work.
`66743fe <https://github.com/plasmodic/ecto/commit/66743fe>`_ Merge branch 'master' of github.com:plasmodic/ecto
`784a27d <https://github.com/plasmodic/ecto/commit/784a27d>`_ what does this do on lucid?
`dd5694f <https://github.com/plasmodic/ecto/commit/dd5694f>`_ valgrind docs tweak
`80000a9 <https://github.com/plasmodic/ecto/commit/80000a9>`_ Merge branch 'master' of github.com:plasmodic/ecto
`fa8fc11 <https://github.com/plasmodic/ecto/commit/fa8fc11>`_ Migrating kitchen docs... still a bit messy.
`c456f1f <https://github.com/plasmodic/ecto/commit/c456f1f>`_ Fix up the python path stuff.
`4542665 <https://github.com/plasmodic/ecto/commit/4542665>`_ tweak fastidious flags for lucid (-Wno-strict-aliasing), as we're getting an error that clearly has nothing todo with the underlying code... there are no pointers being dereferenced anywhere near the location that the compiler is reporting.
`f560a09 <https://github.com/plasmodic/ecto/commit/f560a09>`_ Doc errata gets built if docs are built.
`5bf0f33 <https://github.com/plasmodic/ecto/commit/5bf0f33>`_ Removing a duplicate theme.
`0ded6f2 <https://github.com/plasmodic/ecto/commit/0ded6f2>`_ Merge branch 'master' of github.com:plasmodic/ecto
`b81409b <https://github.com/plasmodic/ecto/commit/b81409b>`_ Remove duplication.
`10a41a7 <https://github.com/plasmodic/ecto/commit/10a41a7>`_ Full overhaul of exception handling. 1.  Use boost::exception 2.  Wrap different exception types on the python side so that they're catchable
`16cce73 <https://github.com/plasmodic/ecto/commit/16cce73>`_ Enable post install tests.
`45216d0 <https://github.com/plasmodic/ecto/commit/45216d0>`_ Moving our sphinx doc directives to our python lib.
`aff2134 <https://github.com/plasmodic/ecto/commit/aff2134>`_ Getting there with standalone docs.
`4502358 <https://github.com/plasmodic/ecto/commit/4502358>`_ Adding xml output to gtest.
`148c100 <https://github.com/plasmodic/ecto/commit/148c100>`_ These tests are not very fun.
`41c2940 <https://github.com/plasmodic/ecto/commit/41c2940>`_ Another pass, why am i so ***
`e4c67aa <https://github.com/plasmodic/ecto/commit/e4c67aa>`_ Woops try to fix it again.
`3b32138 <https://github.com/plasmodic/ecto/commit/3b32138>`_ Fix test on slow build vms.
`a1d1b25 <https://github.com/plasmodic/ecto/commit/a1d1b25>`_ Fixing simlink to __init__.py
`734803a <https://github.com/plasmodic/ecto/commit/734803a>`_ Creating www-deploy target, a bit of doc editing.
`5afb9b1 <https://github.com/plasmodic/ecto/commit/5afb9b1>`_ Comment out the graphviz css crud.
`5ac7113 <https://github.com/plasmodic/ecto/commit/5ac7113>`_ Working on docs in standalone...
`5d2fa73 <https://github.com/plasmodic/ecto/commit/5d2fa73>`_ Moving some stuff around in CMake so i can think.
`cdcf3fc <https://github.com/plasmodic/ecto/commit/cdcf3fc>`_ Adding option for saving a dotfile of the graph.
============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================


amoeba-beta4..amoeba-beta5
^^^^^^^^^^^^^^^^^^^^^^^^^^

============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
`6e1b08a <https://github.com/plasmodic/ecto/commit/6e1b08a>`_ Dependencies,control file, Just install to a normal place.
`c7695a9 <https://github.com/plasmodic/ecto/commit/c7695a9>`_ Adding a multprocess test.
`55ea9fb <https://github.com/plasmodic/ecto/commit/55ea9fb>`_ Adding python original object to the streabuf.
`c299e54 <https://github.com/plasmodic/ecto/commit/c299e54>`_ Working on #191, file like object support.  Tests included.
`c8e780b <https://github.com/plasmodic/ecto/commit/c8e780b>`_ Refer to #188.  Add implementation, test, docs for the Dealer cell. Closes #188.
`e8efd5d <https://github.com/plasmodic/ecto/commit/e8efd5d>`_ Test passes now.
`5fa3026 <https://github.com/plasmodic/ecto/commit/5fa3026>`_ Failing test.
`4f6ef28 <https://github.com/plasmodic/ecto/commit/4f6ef28>`_ remove verbosity.
`c5a0c09 <https://github.com/plasmodic/ecto/commit/c5a0c09>`_ Little clean up options for ipython version and adding logging option.
`37c7f86 <https://github.com/plasmodic/ecto/commit/37c7f86>`_ Remove cruft from tendrils.hpp.
`2b6dd12 <https://github.com/plasmodic/ecto/commit/2b6dd12>`_ Fix up of the python cell, a bit. This needs major work if its to be supported.
`7fd4683 <https://github.com/plasmodic/ecto/commit/7fd4683>`_ rosbuild_lite tweaks.
`9c77fcc <https://github.com/plasmodic/ecto/commit/9c77fcc>`_ Doc option tweak, and CMAKE_BUILD_TYPE correction for standalone.
`0a5902c <https://github.com/plasmodic/ecto/commit/0a5902c>`_ Merge branch 'master' of github.com:plasmodic/ecto
`690ed90 <https://github.com/plasmodic/ecto/commit/690ed90>`_ Working on debian of ecto.
`2d2b524 <https://github.com/plasmodic/ecto/commit/2d2b524>`_ Rosbuild lite messages had no type, so showed up as warnings.
`e2eac41 <https://github.com/plasmodic/ecto/commit/e2eac41>`_ Revert force.
`5120ef3 <https://github.com/plasmodic/ecto/commit/5120ef3>`_ Force the doc target.
`d7e4149 <https://github.com/plasmodic/ecto/commit/d7e4149>`_ Adding cell options.
============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================


amoeba-beta3..amoeba-beta4
^^^^^^^^^^^^^^^^^^^^^^^^^^

============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
`ab7acf3 <https://github.com/plasmodic/ecto/commit/ab7acf3>`_ tweaks for tests running on very, very slow buildbot vms
`6a55e9e <https://github.com/plasmodic/ecto/commit/6a55e9e>`_ this test is failing on the buildbots, want to see what the actual numbers are on a heavily loaded VM
`773cad9 <https://github.com/plasmodic/ecto/commit/773cad9>`_ Disable doc install.
`b5fc2a6 <https://github.com/plasmodic/ecto/commit/b5fc2a6>`_ Adding a virtualenv option.
`0477e10 <https://github.com/plasmodic/ecto/commit/0477e10>`_ This is a major hack... FIXME disable test, refer  #182
`4146c99 <https://github.com/plasmodic/ecto/commit/4146c99>`_ handle when the error is not fatal in rospack. e.g. [rospack] Failed to change UID; cache permissions may need to be adjusted manually. setuid(): Operation not permitted
`3958187 <https://github.com/plasmodic/ecto/commit/3958187>`_ Making rosbuild lite work without sourcing a setup.sh.
`c10221b <https://github.com/plasmodic/ecto/commit/c10221b>`_ More #182
`a45b284 <https://github.com/plasmodic/ecto/commit/a45b284>`_ Attempt to address issue #182.
`9588f26 <https://github.com/plasmodic/ecto/commit/9588f26>`_ Adding install for docs.
`e7e30c9 <https://github.com/plasmodic/ecto/commit/e7e30c9>`_ Better permissions on the deploy.
`e8e3d85 <https://github.com/plasmodic/ecto/commit/e8e3d85>`_ Making docs a bit more flexible, adding deploy cache variables.
`bc7a003 <https://github.com/plasmodic/ecto/commit/bc7a003>`_ Doc tweak.
`4dfa6d3 <https://github.com/plasmodic/ecto/commit/4dfa6d3>`_ Install targets.
`f5b2722 <https://github.com/plasmodic/ecto/commit/f5b2722>`_ Minor virtualenv fixes.
`65c6b46 <https://github.com/plasmodic/ecto/commit/65c6b46>`_ Adding virtual env stuffs.
`64d8af7 <https://github.com/plasmodic/ecto/commit/64d8af7>`_ Quiet on the cmake flags front. Fix fastidious, this wasn't getting picked up... Also, working on install targets for ecto modules.
`490dad1 <https://github.com/plasmodic/ecto/commit/490dad1>`_ tabs to spaces...
`9ca6b30 <https://github.com/plasmodic/ecto/commit/9ca6b30>`_ Reverse order of rsync in docs, so that kitchen can override them.
`fbacb77 <https://github.com/plasmodic/ecto/commit/fbacb77>`_ Merge branch 'master' of git://github.com/plasmodic/ecto
`344bd64 <https://github.com/plasmodic/ecto/commit/344bd64>`_ provoking the buildbots
`293c989 <https://github.com/plasmodic/ecto/commit/293c989>`_ provoking the buildbots
`f63e6d6 <https://github.com/plasmodic/ecto/commit/f63e6d6>`_ provoking the buildbots
`83b5905 <https://github.com/plasmodic/ecto/commit/83b5905>`_ provoking the buildbots
`fbe31e1 <https://github.com/plasmodic/ecto/commit/fbe31e1>`_ provoking the buildbots
`39901c4 <https://github.com/plasmodic/ecto/commit/39901c4>`_ provoking the buildbots
`368595c <https://github.com/plasmodic/ecto/commit/368595c>`_ provoking the buildbots
`12f6745 <https://github.com/plasmodic/ecto/commit/12f6745>`_ provoking the buildbots
`6457588 <https://github.com/plasmodic/ecto/commit/6457588>`_ provoking the buildbots
`e3c2113 <https://github.com/plasmodic/ecto/commit/e3c2113>`_ provoking the buildbots
`e229b46 <https://github.com/plasmodic/ecto/commit/e229b46>`_ Merge branch 'master' of github.com:plasmodic/ecto
`c2f7290 <https://github.com/plasmodic/ecto/commit/c2f7290>`_ provoking the buildbots
`afa306c <https://github.com/plasmodic/ecto/commit/afa306c>`_ Readme touch.
`e6d0c69 <https://github.com/plasmodic/ecto/commit/e6d0c69>`_ Removing the ecto_kitchen docs from ecto, put the in ecto_kitchen.
`df3d559 <https://github.com/plasmodic/ecto/commit/df3d559>`_ doxygen complains about this
`c0dcefa <https://github.com/plasmodic/ecto/commit/c0dcefa>`_ provoking the buildbots
`ba2d0f8 <https://github.com/plasmodic/ecto/commit/ba2d0f8>`_ provoking the buildbots
`5756225 <https://github.com/plasmodic/ecto/commit/5756225>`_ provoking the buildbots
`582ca27 <https://github.com/plasmodic/ecto/commit/582ca27>`_ provoking the buildbots
`0c1edd1 <https://github.com/plasmodic/ecto/commit/0c1edd1>`_ provoking the buildbots
`b658ee0 <https://github.com/plasmodic/ecto/commit/b658ee0>`_ provoking the buildbots
`1a28085 <https://github.com/plasmodic/ecto/commit/1a28085>`_ provoking the buildbots
`0ef52d9 <https://github.com/plasmodic/ecto/commit/0ef52d9>`_ provoking the buildbots
`888f30c <https://github.com/plasmodic/ecto/commit/888f30c>`_ provoking the buildbots
`3402bec <https://github.com/plasmodic/ecto/commit/3402bec>`_ Merge branch 'master' of github.com:plasmodic/ecto
`72d25e5 <https://github.com/plasmodic/ecto/commit/72d25e5>`_ provoking the buildbots
`77facb5 <https://github.com/plasmodic/ecto/commit/77facb5>`_ Adding space.
`0a28bfa <https://github.com/plasmodic/ecto/commit/0a28bfa>`_ Put in if check to the python path to stop errors when ros does not exist.
`ad55b58 <https://github.com/plasmodic/ecto/commit/ad55b58>`_ Merge branches 'master' and 'amoeba-beta0'
`19c354a <https://github.com/plasmodic/ecto/commit/19c354a>`_ Adding a bit to the python options.
`b15c372 <https://github.com/plasmodic/ecto/commit/b15c372>`_ just provoking the buildbot
`dadd9e9 <https://github.com/plasmodic/ecto/commit/dadd9e9>`_ tweak
`3d3872c <https://github.com/plasmodic/ecto/commit/3d3872c>`_ use long and short git tags...   not sure how this got lost
============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================


amoeba-beta2..amoeba-beta3
^^^^^^^^^^^^^^^^^^^^^^^^^^

============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
`23f1983 <https://github.com/plasmodic/ecto/commit/23f1983>`_ docs on how to get tagged releases
`13d9675 <https://github.com/plasmodic/ecto/commit/13d9675>`_ tweak
============================================================= ================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================


