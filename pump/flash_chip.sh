#!/bin/bash
echo $1
espflash flash "$1"
cargo embed
