#!/bin/bash

source /opt/ros/humble/setup.sh

if [ -z "$1" ]; then
    echo "Usage: $0 <target1 target2 target3..>"
    echo
    exit 1
else
    export SIGNING_TOOL=${SIGNING_TOOL:-Tools/saluki-sec-scripts/ed25519_sign.py}

    # go through all given arguments and build them
    for arg in "$@"; do
        echo "BUILDING ${arg}"

        # extract the middle part of the name between the "_"'s
        NAME=${arg}
        NAME=${NAME%_*}
        NAME=${NAME##*_}

        # for our own HW, use HW specific signing keys. For pixhawks, and icicle board,
        # use the PX4 default test key.
        if [[ $NAME = saluki* ]]
        then
            if [[ "$NAME" == "saluki-nxp93" || "$NAME" == "saluki-micro" || "$NAME" == "saluki-ft" ]]; then
                default_signing_args=Tools/saluki-sec-scripts/test_keys/$NAME/secp384r1_test_key1.pem
            else
                default_signing_args=Tools/saluki-sec-scripts/test_keys/$NAME/ed25519_test_key.pem
            fi

            if [[ "$SIGNING_KEY" == *hsm* ]]; then
                echo "Using HSM for signing"
                export SIGNING_TOOL=Tools/saluki-sec-scripts/sign_by_hsm.py
            else
                echo "Using keyfile script for signing"
                export SIGNING_TOOL=Tools/saluki-sec-scripts/sign_by_keyfile.py
            fi
        else
            default_signing_args=Tools/test_keys/ed25519_test_key.pem
        fi

        signing_args=${SIGNING_ARGS:-$default_signing_args}

        # Remove old build output
        rm -Rf build/${arg}
        # Build
        SIGNING_ARGS=${signing_args} make -j$((`nproc`+1)) ${arg}

        if [ -n "$signing_args" ]; then
            echo "Signing key: $signing_args"
        fi
    done
fi
