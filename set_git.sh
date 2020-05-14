#!/bin/bash

# Set local git config for user
git config user.email "abhishekroyn@gmail.com"
git config user.name "Abhishek Roy"

echo Please verify remote:
git remote -v
echo

echo Please verify branch:
git branch
echo

echo Please verify your credentials:
echo username: `git config user.name`
echo email: `git config user.email`

# Make sure you have the latest version of the repo
echo
git pull
echo
