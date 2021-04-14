#!/bin/bash
BASEDIR=$(dirname "$0")

if [[ $# -eq 0 ]] ; then
    echo 'Bitte geben Sie Ihren Namen an.'
    echo 'z.B. "zipTask1 Meier"'
    exit 1
fi

rm ~/Aufgabe1*.zip
zip -r ~/Aufgabe1_$1.zip $BASEDIR/task1
