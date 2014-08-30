# file: do
# do.sh parameter-completion

_do ()   #  By convention, the function name
{                 #+ starts with an underscore.
  local cur
  # Pointer to current completion word.
  # By convention, it's named "cur" but this isn't strictly necessary.

  COMPREPLY=()   # Array variable storing the possible completions.
  cur=${COMP_WORDS[COMP_CWORD]}

  case "$cur" in
    *)
    COMPREPLY=( $( compgen -W 'f1 f2 f3 open tighten loosen guarded \
        solid_contact avoid_contact maintain_pressure dither \
        cylinder spherical probe pinch fingerwalk dof \                      
		    ' -- $cur ) );;
#   Generate the completion matches and load them into $COMPREPLY array.
#   xx) May add more cases here.
#   yy)
#   zz)
  esac

  return 0
}

complete -F _do -o filenames ./do.sh
rosservice call /reflex/do "$*"