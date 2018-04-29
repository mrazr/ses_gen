#!/bin/bash

cd out/production/ses_gen;java -Xms1G -cp ".:../../../lib/*" cz.fi.muni.xmraz3.Main $*
