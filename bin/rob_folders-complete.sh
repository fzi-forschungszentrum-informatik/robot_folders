##
## Copyright (c) 2024 FZI Forschungszentrum Informatik
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
## THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
## THE SOFTWARE.
##

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
