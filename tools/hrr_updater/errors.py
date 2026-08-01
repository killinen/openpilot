from enum import IntEnum


class ExitCode(IntEnum):
  OK = 0
  NO_UPDATE = 2
  HRR_NOT_DETECTED = 10
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


class HrrUpdaterError(RuntimeError):
  exit_code = ExitCode.PROGRAM_FAILURE


class HrrNotDetected(HrrUpdaterError):
  exit_code = ExitCode.HRR_NOT_DETECTED


class NetworkFailure(HrrUpdaterError):
  exit_code = ExitCode.NETWORK_FAILURE


class ReleaseVerificationError(HrrUpdaterError):
  exit_code = ExitCode.RELEASE_VERIFICATION


class ImageVerificationError(HrrUpdaterError):
  exit_code = ExitCode.IMAGE_VERIFICATION


class UnsafeStateError(HrrUpdaterError):
  exit_code = ExitCode.UNSAFE_STATE


class TransportError(HrrUpdaterError):
  exit_code = ExitCode.TRANSPORT_FAILURE


class BootloaderRefused(HrrUpdaterError):
  exit_code = ExitCode.BOOTLOADER_REFUSED


class BootloaderIncompatible(HrrUpdaterError):
  exit_code = ExitCode.BOOTLOADER_INCOMPATIBLE


class CandidateAuthenticationError(HrrUpdaterError):
  exit_code = ExitCode.CANDIDATE_AUTHENTICATION


class TrialFailure(HrrUpdaterError):
  exit_code = ExitCode.TRIAL_FAILURE


class RollbackOccurred(HrrUpdaterError):
  exit_code = ExitCode.ROLLBACK

