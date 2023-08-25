#!/bin/bash
export PATH=~/anaconda3/bin:~/anaconda3/condabin:$PATH

__conda_setup="$('/home/jiho/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/jiho/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/jiho/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/jiho/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup

conda activate dsp-slam

