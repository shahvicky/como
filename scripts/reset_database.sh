#!/bin/bash

sudo service cron stop;
rm -f ~/como/Dator/db.sqlite3;
rm -f ~/default.cfg ;
python ~/como/Dator/manage.py syncdb --noinput;
sudo service cron start;
