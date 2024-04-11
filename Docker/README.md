# robot_folders development container

This dockerfile is mainly used for testing on clean systems. It is not by any means cleanly
maintained or wants to be documented for end users.

We usually use it for development as such (all run from this project's root directory)

1. Build the container

```bash
docker build -t robot_folders Docker
```

2. Run the container
```bash
docker run -it -v .:/robot_folders --rm robot_folders /usr/bin/zsh # replace with /bin/bash to test
# You can also mount a checkout folder to be persistent with that between runs
docker run -it -v .:/robot_folders ~/checkout_playground:/root/checkout --rm robot_folders /usr/bin/zsh
```

3. Inside the container source robot_folders
```bash
source source_global.sh
```
