#include "astbb.h"
#include "dif_prob.h"
#include "lin_corr.h"
#include "bit_perm_opt.h"

SBOX_O_WRD_t GIFT128_sboxes[32 * 16] =
{
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,

	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe,
	0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8, 0xe
};


//Bit Permutation
void GIFT128_diffusion(DIFF_O_WRD_t * out, DIFF_I_WRD_t * in)
{
	uint64_t H = 0ULL;
	uint64_t L = 0ULL;
	int i;
	for (i = 0; i < 16; i++)
	{
		H = (H << 4) | (in[i] & 0xf);
		L = (L << 4) | (in[i + 16] & 0xf);
	}
	// 41 cycles, 8 masks
	ROL128(&H, &L, H, L, 124);
	bit_permute_step_128(&H, &L, H, L, 0x0000000000000000ULL, 0x0ff00ff00ff00f00ULL, 64);
	bit_permute_step_128(&H, &L, H, L, 0x000f0f00000f0f00ULL, 0x000f0f00000f0f00ULL, 4);
	bit_permute_step_128(&H, &L, H, L, 0x00ff00ff00000000ULL, 0x00ff00ff00000000ULL, 8);
	bit_permute_step_128(&H, &L, H, L, 0x00000000a55a5aa5ULL, 0x00000000a55a5aa5ULL, 32);
	bit_permute_step_128(&H, &L, H, L, 0x00000faf00000f5fULL, 0x0000f0500000f0a0ULL, 16);
	bit_permute_step_128(&H, &L, H, L, 0x000a0055000500aaULL, 0x0055000a00aa0005ULL, 8);
	bit_permute_step_128(&H, &L, H, L, 0x0a0a0f0f05050f0fULL, 0x0f0f0a0a0f0f0505ULL, 4);
	bit_permute_step_128(&H, &L, H, L, 0x0000000000000000ULL, 0x666c99933336ccc9ULL, 64);

	for (i = 15; i >= 0; i--)
	{
		out[i] = (DIFF_O_WRD_t)(H & 0xf);
		H = H >> 4;
		out[i + 16] = (DIFF_O_WRD_t)(L & 0xf);
		L = L >> 4;
	}
}


void GIFT128_inv_diffusion(DIFF_I_WRD_t * out, DIFF_O_WRD_t * in)
{
	uint64_t H = 0ULL;
	uint64_t L = 0ULL;
	int i;
	for (i = 0; i < 16; i++)
	{
		H = (H << 4) | (in[i] & 0xf);
		L = (L << 4) | (in[i + 16] & 0xf);
	}
	// 41 cycles, 8 masks
	ROL128(&H, &L, H, L, 124);
	bit_permute_step_128(&H, &L, H, L, 0x00ff00ff00ff00ffULL, 0x0000000000000000ULL, 8);
	bit_permute_step_128(&H, &L, H, L, 0x0f0f0f0f00000000ULL, 0x0f0f0f0f00000000ULL, 4);
	bit_permute_step_128(&H, &L, H, L, 0x00000ff00000f00fULL, 0x0000000000000000ULL, 16);
	bit_permute_step_128(&H, &L, H, L, 0x00000000f00ff00fULL, 0x000000000ff00ff0ULL, 32);
	bit_permute_step_128(&H, &L, H, L, 0x0000000000000000ULL, 0xffff000000ffff00ULL, 64);
	bit_permute_step_128(&H, &L, H, L, 0x000000ff00000000ULL, 0x0000ff000000ffffULL, 16);
	bit_permute_step_128(&H, &L, H, L, 0x0a0a05050a0a0505ULL, 0x0a0a05050a0a0a0aULL, 4);
	bit_permute_step_128(&H, &L, H, L, 0x0063009c009c0063ULL, 0x0063009c009c00c6ULL, 8);

	for (i = 15; i >= 0; i--)
	{
		out[i] = (DIFF_O_WRD_t)(H & 0xf);
		H = H >> 4;
		out[i + 16] = (DIFF_O_WRD_t)(L & 0xf);
		L = L >> 4;
	}
}

#define GIFT128_DC_KNOWN_BEST_ROUND 23

void GIFT128_set_known_prob_bound(PROB_t * B)
{
	// 0Round :nothing
	B[0] = (PROB_t)0;
	// 1Round : 2^{- 1.4150} Best
	B[1] = (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	// 2Round : 2^{- 3.4150} Best
	B[2] = (PROB_t)-2 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	// 3Round : 2^{- 7.0000} Best
	B[3] = (PROB_t)-7;
	// 4Round : 2^{-11.4150} Best
	B[4] = (PROB_t)-10 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	// 5Round : 2^{-17.0000} Best
	B[5] = (PROB_t)-17;
	// 6Round : 2^{-22.4150} Best
	B[6] = (PROB_t)-21 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	// 7Round : 2^{-28.4150} Best
	B[7] = (PROB_t)-27 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	// 8Round : 2^{-39.0000} Best
	B[8] = (PROB_t)-39;
	// 9Round : 2^{-45.4150} Best
	B[9] = (PROB_t)-44 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//10Round : 2^{-49.4150} Best
	B[10] = (PROB_t)-48 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//11Round : 2^{-54.4150} Best
	B[11] = (PROB_t)-53 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//12Round : 2^{-60.4150} Best
	B[12] = (PROB_t)-59 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//13Round : 2^{-67.8301} Best
	B[13] = (PROB_t)-65 + (PROB_t)log2((PROB_t)6 / (PROB_t)16) + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//14Round : 2^{-79.0000} Best
	B[14] = (PROB_t)-79;
	//15Round : 2^{-85.4150} Best
	B[15] = (PROB_t)-84 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//16Round : 2^{-90.4150} Best
	B[16] = (PROB_t)-89 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//17Round : 2^{-96.4150} Best
	B[17] = (PROB_t)-95 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//18Round : 2^{-103.4150} Best
	B[18] = (PROB_t)-102 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//19Round : 2^{-110.8301} Best
	B[19] = (PROB_t)-108 + (PROB_t)log2((PROB_t)6 / (PROB_t)16) + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//20Round : 2^{-121.4150} Best
	B[20] = (PROB_t)-120 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//21Round : 2^{-126.4150} Best
	B[21] = (PROB_t)-125 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//22Round : 2^{-132.4150} Best
	B[22] = (PROB_t)-131 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//23Round : 2^{-139.4150} Best
	B[23] = (PROB_t)-138 + (PROB_t)log2((PROB_t)6 / (PROB_t)16);
	//24Round : 2^{-118.0000} Best
	B[24] = (PROB_t)-118;
	//25Round : 2^{-122.0000} Best
	B[25] = (PROB_t)-122;
	//26Round : 2^{-128.0000} Best
	B[26] = (PROB_t)-128;
	//27Round : 2^{-132.0000} Best
	B[27] = (PROB_t)-132;
	//28Round : 2^{-138.0000} Best
	B[28] = (PROB_t)-138;
};

#define GIFT128_LC_KNOWN_BEST_ROUND 0
void GIFT128_set_known_corr_bound(CORR_t * B)
{
	// 0 Round :  nothing
	B[0].sign = POSI; B[0].magnitude = (ABS_CORR_t)0;
};

#define GIFT128_NUM_ROUND 40
PROB_t GIFT128_each_round_initial_prob_bound[GIFT128_NUM_ROUND + 1] =
{
	(PROB_t)0, //0-round
	(PROB_t)-4,
	(PROB_t)-4,
	(PROB_t)-8,
	(PROB_t)-12,
	(PROB_t)-18,
	(PROB_t)-23,
	(PROB_t)-30,
	(PROB_t)-40,
};
CORR_t GIFT128_each_round_initial_corr_bound[GIFT128_NUM_ROUND + 1] =
{
	{POSI, (ABS_CORR_t)0}, //0-round
	{POSI, (ABS_CORR_t)-2},
	{POSI, (ABS_CORR_t)-3},
	{POSI, (ABS_CORR_t)-5},
	{POSI, (ABS_CORR_t)-7},
	{POSI, (ABS_CORR_t)-10},
	{POSI, (ABS_CORR_t)-15},
	{POSI, (ABS_CORR_t)-18},
	{POSI, (ABS_CORR_t)-21},
};

SEARCHING_START_OPT_t GIFT128_searching_start_opt =
{
	/*enable_1round_active_map*/
	TRUE,
	/*rotational_symmetric_equivalent*/
	FALSE,
};

SET_INITIAL_BOUND_OPT_t GIFT128_set_initial_bound_opt =
{
	/*enable_set_initial_prob_bound*/
	FALSE,
	/*each_round_initial_prob_bound*/
	GIFT128_each_round_initial_prob_bound,
	/*prob_interval*/
	(PROB_t)-5,
	/*enable_set_initial_corr_bound*/
	FALSE,
	/*each_round_initial_corr_bound*/
	GIFT128_each_round_initial_corr_bound,
	/*corr_interval*/
	{POSI, (ABS_CORR_t)-2.5}
};

DIFFUSION_BOUND_OPT_t GIFT128_diffusion_bound_opt =
{
	/*num_diff_of_partial_diffusion*/
	4,
	/*diff_branch_num*/
	UNKNOWN,
	/*inv_trans_diff_branch_num*/
	UNKNOWN
};

BLK_CIPHER_INFO_t GIFT128 =
{
	/*general information*/
	//algname
	"GIFT128",
	//alg_structure
	SPN,
	//num_round
	GIFT128_NUM_ROUND,
	//block_bit_size
	128,
	//key_bit_size
	128,
	//num_word_in_a_state
	32,

	/*about substitution*/
	//distinct_sbox
	FALSE,
	//sbox_i_word_bit_size
	4,
	//sbox_o_word_bit_size
	4,
	//sboxes
	GIFT128_sboxes,

	/*about diffusion*/
	//diffusion_info [what diffusion is used?(e.g., bit-permutation, matrix(partial), matrix(full))]
	BIT_PERMUTATION,
	/*diff_i_word_bit_size*/
	4,
	/*diff_o_word_bit_size*/
	4,
	//diffusion functions
	//Diffusion(DIFF_O_WRD_t *, DIFF_I_WRD_t *)
	GIFT128_diffusion,
	//Inv_Diffusion(DIFF_I_WRD_t *, DIFF_O_WRD_t *)
	GIFT128_inv_diffusion,
	//Inv_Trans_Diffusion(DIFF_O_WRD_t *, DIFF_I_WRD_t *)
	GIFT128_diffusion,
	//Trans_Diffusion(DIFF_I_WRD_t *, DIFF_O_WRD_t *)
	GIFT128_inv_diffusion,

	/*about bound info*/
	//dc_known_best_round
	GIFT128_DC_KNOWN_BEST_ROUND,
	//Set_Known_Prob_Bound
	GIFT128_set_known_prob_bound,
	//lc_known_best_round
	GIFT128_LC_KNOWN_BEST_ROUND,
	//Set_Known_Corr_Bound
	GIFT128_set_known_corr_bound,
	/*start searching opt ptr*/
	&GIFT128_searching_start_opt,
	&GIFT128_set_initial_bound_opt,
	&GIFT128_diffusion_bound_opt
};


void PY_GIFT128_DC_Prob_Searching(PROB_t * rst, CNT_t num_round, UFLAG_t verbose)
{
	SPN_Prep_Info_For_DC(&GIFT128);
	Use_Predefined_1Round_Active_Maps_GIFT128(&GIFT128);
	SPN_Best_DC_Prob_Search(rst, &GIFT128, 0, num_round, verbose);
}

void PY_GIFT128_LC_Corr_Searching(CORR_t * rst, CNT_t num_round, UFLAG_t verbose)
{
	SPN_Prep_Info_For_LC(&GIFT128);
	Use_Predefined_1Round_Active_Maps_GIFT128(&GIFT128);
	SPN_Best_LC_Corr_Search(rst, &GIFT128, 0, num_round, verbose);
}