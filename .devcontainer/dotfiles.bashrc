# Personal shell config, sourced from ~/.bashrc by devcontainer.json postCreateCommand.
alias ll='ls -alF --color=auto'
alias cls='clear'
alias g='git'
alias gst='git status'
alias gcb='git checkout -b'
alias gpl='git pull'
alias gps='git push'
alias gcm='git commit -m'
alias gc='git clone'
alias gchmain='git checkout main; git pull'
alias ga='git add'
alias gch='git checkout'

# cd into a directory by typing just its name
shopt -s autocd

# mkcd: create a directory and cd into it
mkcd() {
  if [ -z "$1" ]; then echo "Usage: mkcd <directory_name>"; return 1; fi
  mkdir "$1" && cd "$1" || { echo "Failed to create directory '$1'."; return 1; }
}

# gccd: clone a repo and cd into it
gccd() {
  git clone "$1" || return
  cd "$(basename "$1" .git)"
}

# syncupstream: sync a fork's main with upstream
syncupstream() {
  git fetch upstream
  git checkout main
  git merge --ff-only upstream/main
  git push origin main
}

git-clean-merged() {
  git fetch --prune
  git branch --merged origin/main | grep -v -E "^\*|main" | xargs -r git branch -d
}
alias gclean='git-clean-merged'

PS1='\[\e[1;34m\]\w\[\e[0m\] \[\e[32m\]>\[\e[0m\] '
