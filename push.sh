#!/bin/bash

# Stage all files changes.
git add .

# Prompt user for input
echo "Please enter commit message."

# Get user input commit message
read commit_msg

# Commit changes
git commit -m commit_msg

# Push to origin of repository
git push origin head