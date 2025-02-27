package frc.robot.util;

import java.util.Objects;

/* How dare java not have this incredibly useful functionality */
@FunctionalInterface
public interface CentiConsumer<
        A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        AA, AB, AC, AD, AE, AF, AG, AH, AI, AJ, AK, AL, AM, AN, AO, AP, AQ, AR, AS, AT, AU, AV, AW, AX, AY, AZ,
        BA, BB, BC, BD, BE, BF, BG, BH, BI, BJ, BK, BL, BM, BN, BO, BP, BQ, BR, BS, BT, BU, BV, BW, BX, BY, BZ,
        CA, CB, CC, CD, CE, CF, CG, CH, CI, CJ, CK, CL, CM, CN, CO, CP, CQ, CR, CS, CT, CU, CV> {
    
    void accept(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t,
                U u, V v, W w, X x, Y y, Z z, AA aa, AB ab, AC ac, AD ad, AE ae, AF af, AG ag, AH ah, AI ai, AJ aj,
                AK ak, AL al, AM am, AN an, AO ao, AP ap, AQ aq, AR ar, AS as, AT at, AU au, AV av, AW aw, AX ax,
                AY ay, AZ az, BA ba, BB bb, BC bc, BD bd, BE be, BF bf, BG bg, BH bh, BI bi, BJ bj, BK bk, BL bl,
                BM bm, BN bn, BO bo, BP bp, BQ bq, BR br, BS bs, BT bt, BU bu, BV bv, BW bw, BX bx, BY by, BZ bz,
                CA ca, CB cb, CC cc, CD cd, CE ce, CF cf, CG cg, CH ch, CI ci, CJ cj, CK ck, CL cl, CM cm, CN cn,
                CO co, CP cp, CQ cq, CR cr, CS cs, CT ct, CU cu, CV cv);
    
    default CentiConsumer<A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
            AA, AB, AC, AD, AE, AF, AG, AH, AI, AJ, AK, AL, AM, AN, AO, AP, AQ, AR, AS, AT, AU, AV, AW, AX, AY, AZ,
            BA, BB, BC, BD, BE, BF, BG, BH, BI, BJ, BK, BL, BM, BN, BO, BP, BQ, BR, BS, BT, BU, BV, BW, BX, BY, BZ,
            CA, CB, CC, CD, CE, CF, CG, CH, CI, CJ, CK, CL, CM, CN, CO, CP, CQ, CR, CS, CT, CU, CV> 
    andThen(CentiConsumer<? super A, ? super B, ? super C, ? super D, ? super E, ? super F, ? super G, ? super H, 
                        ? super I, ? super J, ? super K, ? super L, ? super M, ? super N, ? super O, ? super P, 
                        ? super Q, ? super R, ? super S, ? super T, ? super U, ? super V, ? super W, ? super X, 
                        ? super Y, ? super Z, ? super AA, ? super AB, ? super AC, ? super AD, ? super AE, ? super AF, 
                        ? super AG, ? super AH, ? super AI, ? super AJ, ? super AK, ? super AL, ? super AM, ? super AN, 
                        ? super AO, ? super AP, ? super AQ, ? super AR, ? super AS, ? super AT, ? super AU, ? super AV, 
                        ? super AW, ? super AX, ? super AY, ? super AZ, ? super BA, ? super BB, ? super BC, ? super BD, 
                        ? super BE, ? super BF, ? super BG, ? super BH, ? super BI, ? super BJ, ? super BK, ? super BL, 
                        ? super BM, ? super BN, ? super BO, ? super BP, ? super BQ, ? super BR, ? super BS, ? super BT, 
                        ? super BU, ? super BV, ? super BW, ? super BX, ? super BY, ? super BZ, ? super CA, ? super CB, 
                        ? super CC, ? super CD, ? super CE, ? super CF, ? super CG, ? super CH, ? super CI, ? super CJ, 
                        ? super CK, ? super CL, ? super CM, ? super CN, ? super CO, ? super CP, ? super CQ, ? super CR, 
                        ? super CS, ? super CT, ? super CU, ? super CV> after) {
        Objects.requireNonNull(after);
        return (a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y, z, 
                aa, ab, ac, ad, ae, af, ag, ah, ai, aj, ak, al, am, an, ao, ap, aq, ar, as, at, au, av, aw, ax, ay, az,
                ba, bb, bc, bd, be, bf, bg, bh, bi, bj, bk, bl, bm, bn, bo, bp, bq, br, bs, bt, bu, bv, bw, bx, by, bz,
                ca, cb, cc, cd, ce, cf, cg, ch, ci, cj, ck, cl, cm, cn, co, cp, cq, cr, cs, ct, cu, cv) -> {
            accept(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y, z,
                   aa, ab, ac, ad, ae, af, ag, ah, ai, aj, ak, al, am, an, ao, ap, aq, ar, as, at, au, av, aw, ax, ay, az,
                   ba, bb, bc, bd, be, bf, bg, bh, bi, bj, bk, bl, bm, bn, bo, bp, bq, br, bs, bt, bu, bv, bw, bx, by, bz,
                   ca, cb, cc, cd, ce, cf, cg, ch, ci, cj, ck, cl, cm, cn, co, cp, cq, cr, cs, ct, cu, cv);
            after.accept(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y, z,
                          aa, ab, ac, ad, ae, af, ag, ah, ai, aj, ak, al, am, an, ao, ap, aq, ar, as, at, au, av, aw, ax, ay, az,
                          ba, bb, bc, bd, be, bf, bg, bh, bi, bj, bk, bl, bm, bn, bo, bp, bq, br, bs, bt, bu, bv, bw, bx, by, bz,
                          ca, cb, cc, cd, ce, cf, cg, ch, ci, cj, ck, cl, cm, cn, co, cp, cq, cr, cs, ct, cu, cv);
        };
    }
}