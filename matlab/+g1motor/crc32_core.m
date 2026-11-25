%> @file crc32_core.m
%> @brief Checksum of the bus.

%> @brief Checksum of the bus: CRC-32 over little-endian 32-bit words.
%>
%> Polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final
%> XOR, each word fed most-significant bit first.
%> @param data uint8 bytes; the length must be a multiple of 4.
%> @return The checksum.
function crc = crc32_core(data)
crc = g1motor_mex('crc', uint8(data));
end
