from enum import IntEnum


class ExitCode(IntEnum):
  OK = 0
  NO_UPDATE = 2
  TRQI_NOT_DETECTED = 10
  NETWORK_FAILURE = 11
  RELEASE_VERIFICATION = 12
  IMAGE_VERIFICATION = 13
  UNSAFE_STATE = 14
  TRANSPORT_FAILURE = 15
  BOOTLOADER_REFUSED = 16
  BOOTLOADER_INCOMPATIBLE = 17
  PROGRAM_FAILURE = 18
  CANDIDATE_AUTHENTICATION = 19
  TRIAL_FAILURE = 20
  ROLLBACK = 21
  FAILED_RELEASE_SUPPRESSED = 22


class TrqiUpdaterError(RuntimeError):
  exit_code = ExitCode.PROGRAM_FAILURE


class TrqiNotDetected(TrqiUpdaterError):
  exit_code = ExitCode.TRQI_NOT_DETECTED


class NetworkFailure(TrqiUpdaterError):
  exit_code = ExitCode.NETWORK_FAILURE


class ReleaseVerificationError(TrqiUpdaterError):
  exit_code = ExitCode.RELEASE_VERIFICATION


class ImageVerificationError(TrqiUpdaterError):
  exit_code = ExitCode.IMAGE_VERIFICATION


class UnsafeStateError(TrqiUpdaterError):
  exit_code = ExitCode.UNSAFE_STATE


class TransportError(TrqiUpdaterError):
  exit_code = ExitCode.TRANSPORT_FAILURE


class TransportTimeout(TransportError):
  pass


class BootloaderRefused(TrqiUpdaterError):
  exit_code = ExitCode.BOOTLOADER_REFUSED


class BootloaderIncompatible(TrqiUpdaterError):
  exit_code = ExitCode.BOOTLOADER_INCOMPATIBLE


class CandidateAuthenticationError(TrqiUpdaterError):
  exit_code = ExitCode.CANDIDATE_AUTHENTICATION


class TrialFailure(TrqiUpdaterError):
  exit_code = ExitCode.TRIAL_FAILURE


class RollbackOccurred(TrqiUpdaterError):
  exit_code = ExitCode.ROLLBACK
