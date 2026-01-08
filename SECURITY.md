# Security Policy

## Supported Versions

The following versions of FEAGI Python SDK are currently supported with security updates:

| Version | Supported |
| ------- | --------- |
| 2.0.x   | Yes       |
| < 2.0   | No        |

## Reporting a Vulnerability

**We take security vulnerabilities seriously.** If you discover a security vulnerability in FEAGI Python SDK, please report it responsibly.

### How to Report

1. **Do NOT** open a public GitHub issue for security vulnerabilities
2. Email security details to: **feagi@neuraville.com**
3. Include the following information:
   - Description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Suggested fix (if available)
   - Your contact information

### What to Expect

- **Initial Response**: Within 48 hours
- **Status Update**: Within 7 days
- **Resolution Timeline**: Depends on severity and complexity
- **Public Disclosure**: After a fix is available and tested

### Responsible Disclosure

We follow responsible disclosure practices:
- We will acknowledge receipt of your report
- We will work with you to understand and validate the issue
- We will provide regular updates on the fix progress
- We will credit you in security advisories (unless you prefer to remain anonymous)
- We will coordinate public disclosure timing with you

## Security Considerations

### Network Communication

FEAGI Python SDK communicates with the FEAGI neural engine via:
- **ZMQ (ZeroMQ)**: Used for high-performance sensorimotor data exchange
- **WebSocket**: Used for real-time bidirectional communication
- **REST API**: Used for engine control and genome/connectome operations

**Security Best Practices:**
- Always use secure network configurations in production
- Configure firewall rules to restrict access to FEAGI ports
- Use VPN or private networks for sensitive deployments
- Consider implementing authentication/authorization layers
- Monitor network traffic for anomalies

### Embedded Device Security

When using FEAGI SDK with embedded devices (ESP32, Arduino, etc.):

- **Serial Communication**: Ensure physical access to serial ports is restricted
- **Firmware Updates**: Use secure update mechanisms
- **Credentials**: Never hardcode API keys or credentials in agent code
- **Configuration**: Store sensitive configuration in secure storage
- **OTA Updates**: Use signed firmware updates when available

### Configuration Security

**Never hardcode sensitive values:**
```python
# BAD - Hardcoded credentials
client = FeagiAgentClient("agent", AgentType.SENSORY)
client.configure(feagi_host="192.168.1.100", api_key="secret-key-123")

# GOOD - Use environment variables or secure config
import os
client.configure(
    feagi_host=os.getenv("FEAGI_HOST"),
    api_key=os.getenv("FEAGI_API_KEY")
)
```

**Configuration File Security:**
- Store `feagi_configuration.toml` with appropriate file permissions (600)
- Use environment variable overrides for sensitive values
- Never commit configuration files with credentials to version control
- Rotate API keys and credentials regularly

### Dependency Security

FEAGI Python SDK depends on several third-party libraries. We:

- Regularly update dependencies to address security vulnerabilities
- Monitor security advisories for all dependencies
- Use dependency pinning where appropriate
- Provide security updates for critical vulnerabilities

**Key Dependencies:**
- `pyzmq>=24.0.0` - Network communication
- `aiohttp>=3.9.0` - Async HTTP client
- `numpy>=1.20.0` - Numerical computations
- `feagi-rust-py-libs` - Rust bindings for performance

**Check for Vulnerabilities:**
```bash
pip install safety
safety check --file requirements.txt
```

### Binary Security

FEAGI SDK includes pre-compiled binaries for multiple platforms. These binaries:

- Are built from source in controlled environments
- Should be verified using checksums when available
- Can be code-signed for additional security (see `BUNDLED_BINARIES.md`)
- Can be rebuilt from source if needed

**Verification:**
- Verify checksums before use
- Only download from official sources (PyPI, GitHub releases)
- Consider building from source for maximum security

### Data Privacy

When processing sensor data (camera, audio, etc.):

- **Local Processing**: Prefer local processing when possible
- **Data Transmission**: Encrypt sensitive data in transit
- **Data Storage**: Use encrypted storage for sensitive data
- **Data Retention**: Implement data retention policies
- **Compliance**: Ensure compliance with applicable privacy regulations (GDPR, CCPA, etc.)

### Agent Security

When developing FEAGI agents:

- **Input Validation**: Always validate and sanitize sensor inputs
- **Output Validation**: Validate motor commands before execution
- **Error Handling**: Implement proper error handling to prevent crashes
- **Resource Limits**: Set appropriate resource limits (CPU, memory, network)
- **Logging**: Avoid logging sensitive data (credentials, personal information)
- **Testing**: Test agents in isolated environments before deployment

## Security Best Practices

### Development

1. **Keep Dependencies Updated**
   ```bash
   pip install --upgrade feagi
   pip list --outdated
   ```

2. **Use Virtual Environments**
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   pip install feagi
   ```

3. **Review Code Changes**
   - Review all code changes before deployment
   - Use code review tools and static analysis
   - Run security linters (bandit, safety)

4. **Secure Configuration Management**
   - Use environment variables for secrets
   - Use secure configuration management tools
   - Rotate credentials regularly

### Deployment

1. **Network Security**
   - Use firewalls to restrict access
   - Implement network segmentation
   - Use VPN or private networks
   - Monitor network traffic

2. **Access Control**
   - Implement authentication/authorization
   - Use least privilege principle
   - Regularly audit access logs
   - Implement rate limiting

3. **Monitoring and Logging**
   - Monitor for suspicious activity
   - Log security-relevant events
   - Set up alerts for anomalies
   - Regularly review logs

4. **Backup and Recovery**
   - Regular backups of configurations
   - Test recovery procedures
   - Store backups securely
   - Document recovery procedures

## Known Security Limitations

### Current Limitations

1. **Authentication**: Basic authentication hooks exist but comprehensive security features are not fully implemented in all components
2. **Encryption**: Optional encryption is available but not enabled by default
3. **ZMQ Security**: ZMQ CURVE encryption is supported but requires explicit configuration
4. **API Security**: REST API authentication is available but may need additional hardening for production use

### Future Security Enhancements

- Enhanced authentication and authorization
- End-to-end encryption for all communications
- Certificate-based authentication
- Security audit logging
- Automated security scanning in CI/CD

## Security Updates

Security updates are released as:
- **Patch releases** (e.g., 2.0.1 → 2.0.2) for critical security fixes
- **Minor releases** (e.g., 2.0.x → 2.1.0) for security enhancements
- **Security advisories** published on GitHub and via email to registered users

**Stay Updated:**
- Watch the repository for security advisories
- Subscribe to release notifications
- Regularly update your installation

## Security Contacts

- **Security Email**: feagi@neuraville.com
- **GitHub Issues**: For non-security bugs and feature requests
- **Documentation**: https://docs.feagi.org

## Security Acknowledgments

We thank security researchers and community members who responsibly disclose vulnerabilities. Contributors will be credited in security advisories (unless they prefer to remain anonymous).

## Additional Resources

- [FEAGI Architecture Documentation](docs/)
- [Contributing Guidelines](CONTRIBUTING.md)
- [Code of Conduct](CONTRIBUTING.md#code-of-conduct)
- [Apache 2.0 License](LICENSE.txt)

---

**Last Updated**: 2025-01-16  
**Maintained by**: Neuraville Inc.

