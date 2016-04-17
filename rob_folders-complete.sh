_rob_folders_completion() {
    COMPREPLY=( $( env COMP_WORDS="${COMP_WORDS[*]}" \
                   COMP_CWORD=$COMP_CWORD \
                   _ROB_FOLDERS_COMPLETE=complete rob_folders ) )
    return 0
}

complete -F _rob_folders_completion -o default fzirob;
complete -F _rob_folders_completion -o default rob_folders;
