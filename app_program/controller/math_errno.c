/** @file math_errno.c
 *
 * @brief Newlib libm requests __errno even though the controller validates math
 * inputs. Supply an error slot without pulling in newlib stdio/heap and OS
 * syscalls. No per-task errno contract: application code must use explicit
 * error results.
 */

/*! @brief Error number storage for the maths library. */
int *
__errno (void)
{
    static int math_error;

    return &math_error;
}

/*** end of file ***/
