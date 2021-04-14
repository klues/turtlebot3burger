#!/bin/sh
BASEDIR=$(dirname "$0")
echo "$BASEDIR"

rm $BASEDIR/abgabe.zip
zip -r $BASEDIR/abgabe.zip $BASEDIR -x $BASEDIR/.git/\* $BASEDIR/doc/\*
