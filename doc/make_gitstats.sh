#!/bin/sh

COMMITS=$(git rev-list --all | wc -l)
MERGES=$(git rev-list --all --merges | wc -l)

echo "$COMMITS commits"
echo "$MERGES  merges"
echo REVS=$(git rev-list --all)

for rev in $(git rev-list --all | head -n -1)
do
    echo "========== $rev ============"
    THISONE=$(git --no-pager diff --numstat $rev..$rev^)
    echo $THISONE
done
