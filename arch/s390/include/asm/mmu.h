#ifndef __MMU_H
#define __MMU_H

typedef struct {
	atomic_t attach_count;
	unsigned int flush_mm;
	spinlock_t list_lock;
	struct list_head pgtable_list;
	unsigned long asce_bits;
	unsigned long asce_limit;
	unsigned long vdso_base;
	/* Cloned contexts will be created with extended page tables. */
	unsigned int alloc_pgste:1;
	/* The mmu context has extended page tables. */
	unsigned int has_pgste:1;
} mm_context_t;

#endif
