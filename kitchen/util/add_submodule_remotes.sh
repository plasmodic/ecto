#!/bin/sh

MY_GITHUB_NAME=$1
if [ "$MY_GITHUB_NAME" = "" ] ; then
    echo "usage: $0 <github username>"
    exit 1
fi
echo "Adding remotes for plasmodic and $MY_GITHUB_NAME"

doit () 
{
    git remote add plasmodic git@github.com:plasmodic/$1.git
    git remote add $MY_GITHUB_NAME git@github.com:$MY_GITHUB_NAME/$1.git
    echo "========= $1 =========="
    git remote -v
}

doit ecto_kitchen

cd ecto
doit ecto
cd ..

for proj in opencv pcl ros
do
    cd $proj
    doit ecto_$proj
    cd ..
done

