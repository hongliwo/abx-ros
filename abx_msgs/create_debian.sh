#! /usr/bin/env bash


###############################################
## Possible solutions for common errors
## sudo rosdep init
## rosdep update

## if "CERTIFICATE_VERIFY_FAILED",
## run "export SSL_CERT_FILE=/usr/lib/ssl/certs/ca-certificates.crt" at the command line

## If encouter "dpkg-shlibdeps: error: no dependency information found for" Error
## then "dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info"
###############################################


echo -e "\n\033[1;32m ~~ (1). Delete old debian folders in the directory...\033[0m"
#rm -rf debian/ obj-aarch64-linux-gnu/

echo -e "\n\033[1;32m ~~ (2). Delete any backup files...\033[0m"
find . -type f -name '*~' -delete

echo -e "\n\033[1;32m ~~ (3). Create debian packages...\033[0m\n"
bloom-generate rosdebian --os-name ubuntu --os-version `echo $(lsb_release -sc)` --ros-distro `echo ${ROS_DISTRO}` && fakeroot debian/rules binary

echo -e "\n\033[1;32m ~~ (4). Delete debian folders in the directory...\033[0m"
#rm -rf debian/ obj-aarch64-linux-gnu/
