_fzrob_completion() {
    COMPREPLY=( $( env COMP_WORDS="${COMP_WORDS[*]}" \
                   COMP_CWORD=$COMP_CWORD \
                   _FZROB_COMPLETE=complete $1 ) )
    return 0
}

complete -F _fzrob_completion -o default fzrob;
