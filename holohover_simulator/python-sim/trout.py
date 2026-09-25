import numpy as np

# a = np.array([1, 2, 3])

# k = np.where(np.any(a > 1), "ja", "nein")
# print(k)


# b = np.array([[1, 2], [3, 4], [5, 6]])
# c = np.array([[7], [8], [9]])

# d = np.hstack((b, c))
# print(d)
# print(d.shape)

# print(c.shape)
# e = np.repeat(c, 3, axis=1)
# print(e)

# f = np.array([1, 2, 3])
# print(f.shape)

# g, h, i = f
# print(g)

# unique_seqs, inv_idx = jnp.unique(wall_bounce_seq, axis=0, return_inverse=True)
# group_min_cost = jax.ops.segment_min(costs, inv_idx, num_segments=unique_seqs.shape[0])
# best_group = jnp.argmin(group_min_cost)
# mask = inv_idx == best_group

# masked_costs = jnp.where(mask, costs, jnp.inf)
# weights = jnp.exp(-masked_costs/temperature)
# weights = jnp.where(mask, weights, 0.0)
# weights /= jnp.sum(weights)

# h = np.array([[1, 0], [1, 0], [0, 0], [0, 0], [1, 1]])
# unique_seqs, inv_idx = np.unique(h, axis=0, return_inverse=True)

# print(unique_seqs)
# print(inv_idx)

mask = np.array([0, 1, 1, 0, 0, 0])
costs = np.array([1, 2, 3, 4, 5, 6])

masked_costs = np.where(mask, costs, np.inf)
weights = np.exp(-masked_costs)
weights = np.where(mask, weights, 0.0)
weights /= np.sum(weights)
print(weights)

weights = np.where(mask, np.exp(-costs), 0.0)
weights /= np.sum(weights)
print(weights)

a = np.array([1, 2, 3, 4, 5])
b = a[:3]
print(a.shape)
print(b.shape)

c = np.array([-1, -1, -1])
indices = np.where(c > 0)[0]
# first_index_non_neg = indices[0][0] if indices[0].size > 0 else -1
first_index_non_neg = np.argmax(c > 0)
print(c > 0)
print(first_index_non_neg)

costs = np.array([1e10, 1e10, 3e10, 2, 1000, 10000])
temperature = 1e6
weights = np.exp(-costs / temperature)
weights /= np.sum(weights)
print(weights)
