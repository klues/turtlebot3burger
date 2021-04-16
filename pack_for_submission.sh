#!/bin/bash
BASEDIR=$(dirname "$0")

if [[ $# -le 1 ]] ; then
    echo 'Verwendung: zipTask <TaskNr> <Name(n)>'
    echo 'Bitte geben Sie Task-Nummer und Ihre Namen an.'
    echo 'z.B. "zipTask 1 Meier_Muster"'
    exit 1
fi

rm ~/Aufgabe$1*.zip
zip -r ~/Aufgabe$1_$2.zip $BASEDIR -x $BASEDIR/.git/\* $BASEDIR/doc/\*
echo "Datei ~/Aufgabe$1_$2.zip erfolgreich erstellt!"

# add to .bashrc
#~ function zipTask() {
    #~ cd ~/catkin_ws/src/turtlebot3burger
    #~ bash pack_for_submission.sh "$@"
    #~ cd - > /dev/null
#~ }
