In this folder there will be scripts that are workspace-independent
and will be needed for this robot by any user.

This includes the choose_environment.bash that should also be available 
via an alias (ce). Other possible software could be executables to 
configure hardware or make some diagnosis.

You should add the following lines to your .bashrc:

 # add an alias that sources the choose_environment.bash
 alias ce='source $HOME/bin/choose_environment.bash'

 # delete the .cmake folder since it is not working with mutliple workspaces
 cd $HOME/.cmake/packages && rm -rf *

