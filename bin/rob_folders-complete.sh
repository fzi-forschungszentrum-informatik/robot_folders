
if [ -n "${ZSH_VERSION+1}" ];
then
  compdef _rob_folders_completion fzirob
fi
if [ -n "${BASH_VERSION+1}" ];
then
  _fzirob_completion() {
    # replace 'fzirob' with 'rob_folders' for completion
    _rob_folders_completion rob_folders ${@:2}
    return 0
  }
  complete -F _fzirob_completion -o default fzirob
fi
#complete -F _rob_folders_completion -o default rob_folders;

# ce completion in zsh works through the alias definition. For bash we have to declare it.
if [ -n "${BASH_VERSION+1}" ];
then
  _ce_completion() {
    checkout_dir=$(rob_folders get_checkout_base_dir)
    #_envs=$(ls ${checkout_dir})
    _envs=$(ce --help | sed -e '1,/Commands:/d' )
    local cur prev
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    COMPREPLY=( $(compgen -W "${_envs}" -- ${cur}) )
    return 0
  }

  complete -F _ce_completion -o default ce;
fi
