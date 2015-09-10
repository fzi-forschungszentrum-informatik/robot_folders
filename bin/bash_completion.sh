# auto completion for ce script

_script()
{
  _script_commands=$(choose_environment.bash shortlist)

  local cur prev
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "${_script_commands}" -- ${cur}) )

  return 0
}

# map the auto-completion of the ce alias
complete -o nospace -F _script ce
