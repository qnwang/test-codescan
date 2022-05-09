#include <stdio.h>
#include <stdlib.h>
#include <curl/curl.h>

#include "jitsealapi.h"
#include "jitinterface.h"


static CURLcode sslctx_function(CURL *curl, void *sslctx, void *parm)
{

	printf("sslctx_function\n");

	int ret = jitiot_se_sslctx_function(curl, sslctx, parm);
	printf("jitiot_se_sslctx_function ret=%d\n", ret);
	return CURLE_OK;
}

static size_t writefunction(void *ptr, size_t size, size_t nmemb, void *stream)
{
	printf("writefunction, size=%d\n", (int)size);
	fwrite(ptr, size, nmemb, (FILE *)stream);
	return (nmemb*size);
}

int curl_jit_tls_test(void)
{
	CURL *ch;
	curl_global_init(CURL_GLOBAL_ALL);
	ch = curl_easy_init();
	curl_easy_setopt(ch, CURLOPT_VERBOSE, 1L);
//	curl_easy_setopt(ch, CURLOPT_HEADER, 0L);
//	curl_easy_setopt(ch, CURLOPT_NOPROGRESS, 1L);
//	curl_easy_setopt(ch, CURLOPT_NOSIGNAL, 1L);
	curl_easy_setopt(ch, CURLOPT_WRITEFUNCTION, *writefunction);
	curl_easy_setopt(ch, CURLOPT_WRITEDATA, stdout);
	curl_easy_setopt(ch, CURLOPT_HEADERFUNCTION, *writefunction);
	curl_easy_setopt(ch, CURLOPT_HEADERDATA, stderr);
	curl_easy_setopt(ch, CURLOPT_SSL_VERIFYPEER, 1);
	curl_easy_setopt(ch, CURLOPT_SSL_VERIFYHOST, 2);
	curl_easy_setopt(ch, CURLOPT_SSLVERSION, 1); //1 3

	/*use fixed-pin
	char *pin = "jitiot123456";
	curl_easy_setopt(ch, CURLOPT_SSL_CTX_DATA, pin);*/

#if 1
	char* pki_ca_root = "/oemapp/smartlink/default/security/test_root_cert.pem";
	char* pki_cert_path = "/userdata/smartlink/config/pki/";
	char* pki_key_path = "/userdata/smartlink/config/pki/";
	char buff[32] = {0};
	char* pki_pin = buff;
	int pki_len = 0;

	FILE *fp = NULL;

   	fp = fopen("/userdata/smartlink/config/pki/pki.pin", "r");
	if (fp)
	{
		fgets(pki_pin, 255, (FILE*)fp);
		fclose(fp);
	}


#endif

	curl_easy_setopt(ch, CURLOPT_SSL_CTX_DATA, pki_pin);
//	curl_easy_setopt(ch, CURLOPT_CAPATH, "/app/ota/excelfore/ca/");
	curl_easy_setopt(ch, CURLOPT_CAINFO, pki_ca_root);
//	curl_easy_setopt(ch, CURLOPT_SSLCERT, "/app/ota/excelfore/cert.pem");
//	curl_easy_setopt(ch, CURLOPT_SSLCERTTYPE, "PEM");
//	curl_easy_setopt(ch, CURLOPT_SSLKEY, "/app/ota/excelfore/client.key");
	curl_easy_setopt(ch, CURLOPT_URL, "https://otatest.fawjiefang.com.cn:8443/snap/oma");
//	curl_easy_setopt(ch, CURLOPT_URL, "https://ota.fawjiefang.com.cn");
//	curl_easy_setopt(ch, CURLOPT_URL, "https://117.78.52.36:8443/snap/oma");
//	curl_easy_setopt(ch, CURLOPT_URL, "http://221.229.116.134:12580");
//	curl_easy_setopt(ch, CURLOPT_URL, "https://140.143.150.75:38000");
//	curl_easy_setopt(ch, CURLOPT_SSL_CTX_FUNCTION, *sslctx_function);

	CURLcode rv = curl_easy_setopt(ch, CURLOPT_SSL_CTX_FUNCTION, *sslctx_function);
	if (rv == CURLE_OK) {
		printf("TLS set OK\n");
	} else {
		printf("TLS failed to set\n");
	}
#if 0
	jit_seal_default_opt opt = {0};
	opt.conn_func = jit_seal_SE_conn;
	opt.disc_func = jit_seal_SE_disc;
	opt.genkey_func =jit_seal_gen_asymm_keypair;
	opt.getpubkey_func = jit_seal_get_pubkey;
	opt.sign_func = jit_seal_sign;
	opt.dec_func = jit_seal_decrypt;
	jit_seal_assign_default(&opt);
	opt.conn_func();
#endif
	int ret = jitiot_set_path(pki_key_path, pki_cert_path);
	printf("jitiot_set_path ret=%d\n", ret);
	ret = jitiot_set_caname(pki_ca_root);
	printf("jitiot_set_caname ret=%d\n", ret);

	rv = curl_easy_perform(ch);
	if(rv == CURLE_OK) {
		printf("\nTLS connected OK");
	} else {
		printf("\nTLS failed to connect, rv=%d: %s\n", rv, curl_easy_strerror(rv));
	}

	curl_easy_cleanup(ch);
	curl_global_cleanup();

	return 0;
}

int main(int argc, char **argv)
{
	int ret = 0;
	curl_jit_tls_test();

	exit(0);
}
