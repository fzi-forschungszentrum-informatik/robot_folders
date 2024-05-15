Installation
============

It is recommended to install robot_folders using ``pipx`` (You can install ``pipx`` on Ubuntu using ``sudo apt
install pipx``). Please note: With pipx versions < 1.0 you'll have to provide the `--spec` flag.

.. code:: bash

   pipx install robot_folders

Upgrade
-------

To upgrade robot_folders using ``pipx`` do

.. code:: bash

   pipx upgrade robot_folders

Shell setup
-----------

In order to use ``robot_folders`` you'll have to source its source file. The path of the source file
depends on the installation method:

.. tabs::

   .. tab:: pipx < 1.3.0

      .. code:: bash

         # pipx < 1.3.0 or if ${HOME}/.local/pipx already existed
         echo "source ${HOME}/.local/pipx/venvs/robot-folders/bin/rob_folders_source.sh" >> ~/.bashrc

   .. tab:: pipx >= 1.3.0

      Note: When coming from older pipx versions the path ``${HOME}/.local/pipx/venvs`` might
      already exist, in which case newer pipx versions will use that one. In that case you'll have
      to use the source instructions from pre-1.3.0.

      .. code:: bash

         echo "source ${HOME}/.local/share/pipx/venvs/robot-folders/bin/rob_folders_source.sh" >> ~/.bashrc
   .. tab:: pip venv

      In case you manually installed ``robot_folders`` using a python virtualenv the path is similarly

      .. code:: bash

         echo "source <your-virtualenv-path>/bin/rob_folders_source.sh" >> ~/.bashrc
