#! /usb/bin/env sh
#
# author  Ralf Kohlhaas <kohlhaas@fzi.de>
# date    2015-01-23
#
# Do not call this script directly, but use the bash or zsh version respectively
#----------------------------------------------------------------------

# ==================================================
#   Default Variables
# ==================================================

if [ -z ${SCRIPT_DIR+x} ]; then
    echo "SCRIPT_DIR is unset. Please call the .bash or .zsh version of this script"
else
    echo "SCRIPT_DIR is set to '$SCRIPT_DIR'"

    CHECKOUT_DIR=$( readlink -e $SCRIPT_DIR/../checkout )

    # add ~/bin to PATH
    export PATH=$SCRIPT_DIR:$PATH

    # delete the .cmake folder since it is not working with mutliple workspaces
    rm -rf $HOME/.cmake/packages/
fi
