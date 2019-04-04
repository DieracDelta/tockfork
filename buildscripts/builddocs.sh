source ./paths.sh

cd $BOARD_PATH
export TOCK_KERNEL_VERSION=$(git describe --always | echo notgit)
make doc
