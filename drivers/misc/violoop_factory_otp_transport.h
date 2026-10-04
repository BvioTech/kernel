/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _VIOLOOP_FACTORY_OTP_TRANSPORT_H
#define _VIOLOOP_FACTORY_OTP_TRANSPORT_H
#include <linux/errno.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>

static int violoop_otp_match(struct tee_ioctl_version_data *v, const void *unused)
{
	return v->impl_id == TEE_IMPL_ID_OPTEE;
}

/* RK storage PTA. Only the first aligned OEM word is ever addressed. */
static int violoop_otp_transfer(__le32 *word, bool write)
{
	const uuid_t uuid = UUID_INIT(0x2d26d8a8, 0x5134, 0x4dd8,
		0xb3, 0x2f, 0xb3, 0x4b, 0xce, 0xeb, 0xc4, 0x71);
	struct tee_ioctl_open_session_arg session = {};
	struct tee_ioctl_invoke_arg invoke = {};
	struct tee_param p[4] = {};
	struct tee_context *ctx;
	struct tee_shm *shm;
	__le32 *shared;
	int ret;

#ifndef VIOLOOP_OTP_WRITE_MODULE
	if (write)
		return -EPERM;
#endif
	ctx = tee_client_open_context(NULL, violoop_otp_match, NULL, NULL);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	memcpy(session.uuid, uuid.b, TEE_IOCTL_UUID_LEN);
	session.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	ret = tee_client_open_session(ctx, &session, NULL);
	if (ret || session.ret) {
		if (!ret)
			ret = -EIO;
		goto out_ctx;
	}
	shm = tee_shm_alloc_kernel_buf(ctx, sizeof(*word));
	if (IS_ERR(shm)) {
		ret = PTR_ERR(shm);
		goto out_session;
	}
	shared = tee_shm_get_va(shm, 0);
	if (IS_ERR(shared)) {
		ret = PTR_ERR(shared);
		goto out_shm;
	}
	*shared = write ? *word : 0;
#ifdef VIOLOOP_OTP_WRITE_MODULE
	invoke.func = write ? 12 : 13;
#else
	invoke.func = 13;
#endif
	invoke.session = session.session;
	invoke.num_params = ARRAY_SIZE(p);
	p[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	p[0].u.value.a = 0;
	p[1].attr = write ? TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT :
		TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	p[1].u.memref.shm = shm;
	p[1].u.memref.size = sizeof(*word);
	ret = tee_client_invoke_func(ctx, &invoke, p);
	if (!ret && (invoke.ret || p[1].u.memref.size != sizeof(*word)))
		ret = -EIO;
	if (!ret && !write)
		*word = *shared;
	memzero_explicit(shared, sizeof(*word));
out_shm:
	tee_shm_free(shm);
out_session:
	tee_client_close_session(ctx, session.session);
out_ctx:
	tee_client_close_context(ctx);
	return ret;
}
#endif
