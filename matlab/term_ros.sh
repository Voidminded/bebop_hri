#!/bin/bash
ps aux | grep ros | tr -s " " | cut -d " " -f 2 | xargs kill -INT
