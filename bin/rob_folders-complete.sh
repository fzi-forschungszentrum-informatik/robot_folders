_rob_folders_completion() {
    COMPREPLY=( $( env COMP_WORDS="${COMP_WORDS[*]}" \
                   COMP_CWORD=$COMP_CWORD \
                   _ROB_FOLDERS_COMPLETE=complete rob_folders ) )
    return 0
}

complete -F _rob_folders_completion -o default fzirob;
complete -F _rob_folders_completion -o default rob_folders;

# ce completion in zsh works through the alias definition. For bash we have to declare it.
if [ -n "${BASH_VERSION+1}" ];
then
    _ce_completion() {
        _envs=$(ls ${ROB_FOLDERS_BASE_DIR}/checkout)
        local cur prev
        COMPREPLY=()
        cur="${COMP_WORDS[COMP_CWORD]}"
        COMPREPLY=( $(compgen -W "${_envs}" -- ${cur}) )
        return 0
    }

    complete -F _ce_completion -o default ce;
fi
