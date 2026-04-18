#pragma once

#include <Arduino.h>

namespace identity_service {

String normalizeBackendUrl(const String &url);
String buildValidationUrl(const String &backendUrl);
bool requestUuidFromBackend(const String &backendUrl,
                            const String &email,
                            const String &accountPassword,
                            String &uuidOut,
                            String &errorOut);
String generateAnonymousUuid();

}  // namespace identity_service
