#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="10.1"
fi

export STAGING_ROOT="/data/safe_staging"

eval "$(/data/openpilot/frogpilot/system/environment_variables)"

export SENTRY_DSN="https://600a64896ab85d512cd942bf7dac1984@o1107536.ingest.us.sentry.io/4510260793114624"
export SENTRY_DSN_NATIVE="https://600a64896ab85d512cd942bf7dac1984@o1107536.ingest.us.sentry.io/4510260793114624"
# This is for teletyped sentry if want to use different endpoint than the "main"
export GCS_SENTRY_DSN="https://82a4222b21bdd8e738c0f20677110918@o1107536.ingest.us.sentry.io/4509169784848384"
