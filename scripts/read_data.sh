#!/bin/bash

while :
do


read -r -p "Are you sure? [1/0] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
        echo "yes"
        ;;
    *)
      	echo "no"
        ;;
esac
done
