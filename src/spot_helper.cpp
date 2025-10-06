#include "erl_env/spot_helper.hpp"

#include "erl_common/logging.hpp"

namespace erl::env::spot_helper {

    // inspired by bdd_printset_rec in buddy/src/cppext.cxx
    static void
    bdd_to_vec_rec(
        std::vector<int> &vec,
        const bdd &r,
        std::vector<int> &set) {        // NOLINT(*-no-recursion)
        if (r == bddfalse) { return; }  // false terminal
        if (r == bddtrue) {             // true terminal, dump to vec
            for (int n = 0; n < bdd_varnum(); ++n) {
                if (set[n] > 0) {
                    const int var = bdd_level2var(n);
                    vec[var] = set[n] == 2;
                }
            }
        } else {
            // position of the variable in the current variable order
            const int level = bdd_var2level(bdd_var(r));
            set[level] = 1;
            bdd_to_vec_rec(vec, bdd_low(r), set);
            set[level] = 2;
            bdd_to_vec_rec(vec, bdd_high(r), set);
            set[level] = 0;
        }
    }

    std::vector<int>
    BddToFlags(const bdd &b) {
        const int varnum = bdd_varnum();
        std::vector<int> vec(varnum, kDONT_CARE);
        if (b == bddtrue) {
            vec.resize(varnum, kTRUE);
            return vec;
        }
        if (b == bddfalse) {
            vec.resize(varnum, kFALSE);
            return vec;
        }
        std::vector<int> set(varnum, 0);
        int level = bdd_var2level(bdd_var(b));
        set[level] = 1;
        bdd_to_vec_rec(vec, bdd_low(b), set);
        set[level] = 2;
        bdd_to_vec_rec(vec, bdd_high(b), set);
        set[level] = 0;
        return vec;
    }

}  // namespace erl::env::spot_helper
