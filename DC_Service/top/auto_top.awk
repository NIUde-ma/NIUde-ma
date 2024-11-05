#!/bin/bash
top_params="-b -d 1 -c"
while :
do
    unbuffer top $top_params | awk 'BEGIN { OFS="\t" } NR>7 {print $1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12}'
    echo "$result"
    sleep 1
done