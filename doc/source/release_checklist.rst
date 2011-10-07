Release Checklist
=================

* Be sure you are fully merged::
  
    git submodule foreach git fetch --all
    git submodule foreach git pull plasmodic master

* Verify testing.

* Pick your tag.  We'll call this one ``amoeba-0``

* Tweak version information in ecto/cmake/version.hpp

* If desired update ABI info in ecto/include/ecto/abi.hpp

* Add to list in ecto/doc/source/changelog_gen.py

* Tag kitchen and projects with the same tag::

    % git submodule foreach git tag amoeba-0
    Entering 'ecto'
    Entering 'opencv'
    Entering 'openni'
    Entering 'pcl'
    Entering 'ros'
    % git tag amoeba-0

* Make docs, verify they look good.
    
* Change tag/link on main ecto page to latest release




