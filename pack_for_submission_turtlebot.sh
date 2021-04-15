#!/bin/bash
BASEDIR=$(dirname "$0")

if [[ $# -eq 0 ]] ; then
    echo 'Bitte geben Sie Ihren Namen an.'
    echo 'z.B. "zipTasks Meier"'
    exit 1
fi

rm ~/Aufgaben2ff*.zip
zip -r ~/Aufgaben2ff_$1.zip $BASEDIR -x $BASEDIR/.git/\* $BASEDIR/doc/\* $BASEDIR/task1/\*
echo "Datei ~/Aufgaben2ff_$1.zip erfolgreich erstellt!"
