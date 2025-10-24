#!/bin/bash

source /opt/ros/humble/setup.sh

if [ -z "$1" ]; then
    echo "Usage: $0 <target1 target2 target3..>"
    echo
    exit 1
else
    # go through all given arguments and build them
    for arg in "$@"; do
        echo "BUILDING ${arg}"

        # extract the middle part of the name between the "_"'s
        NAME=${arg}
        NAME=${NAME%_*}
        NAME=${NAME##*_}

        # for our own HW, use HW specific siging. For pixhawks, and icicle board,
        # use the PX4 default signing script and keys
        if [[ $NAME = saluki* ]]
        then
            if [ -z "$SIGNING_ARGS" ]; then
                export SIGNING_ARGS=Tools/saluki-sec-scripts/test_keys/$NAME/secp384r1_test_key0.pem
            fi

            if [[ -z "$SIGNING_KEY" && "$NAME" == "saluki-nxp93" ]]; then
               echo "Using secp384r1_sign.py script for signing"
               export SIGNING_TOOL=Tools/saluki-sec-scripts/secp384r1_sign.py
	    elif [[ "$SIGNING_KEY" = "hsm" ]]; then
               echo "Using HSM for signing"
               export SIGNING_TOOL=Tools/saluki-sec-scripts/sign_hsm.py
            else
               echo "Using ed25519_sign.py script for signing"
               export SIGNING_TOOL=Tools/saluki-sec-scripts/ed25519_sign.py
            fi
        else
            export SIGNING_TOOL=Tools/cryptotools.py
            unset SIGNING_ARGS
        fi

        # Remove old build output
        rm -Rf build/${arg}
        # Build
        make -j$((`nproc`+1)) ${arg}

        if [ -n "$SIGNING_ARGS" ]; then
            echo "Signing key: $SIGNING_ARGS"
        fi
    done
fi
