#/usr/bin/env bash

doxygen docs/Doxyfile 2&> out.out
cat out.out | grep "not documented" > notDoc.out
cat notDoc.out