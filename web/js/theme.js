/**
 * Theme Toggle Script
 * Handles light/dark theme switching and persistence
 * Light theme is default, Dark theme uses data-theme="dark"
 */

(function() {
    'use strict';

    function applyTheme(theme) {
        if (theme === 'dark') {
            document.documentElement.setAttribute('data-theme', 'dark');
        } else {
            document.documentElement.removeAttribute('data-theme');
        }
    }

    function toggleTheme() {
        var currentTheme = localStorage.getItem('crawler-theme') || 'light';
        var newTheme = currentTheme === 'light' ? 'dark' : 'light';

        applyTheme(newTheme);
        localStorage.setItem('crawler-theme', newTheme);

        // Update checkbox state
        var themeCheckbox = document.getElementById('theme-checkbox');
        if (themeCheckbox) {
            themeCheckbox.checked = (newTheme === 'dark');
        }

        console.log('[theme.js] Theme toggled to:', newTheme);
    }

    // Initialize theme on page load (before DOMContentLoaded for faster apply)
    function initTheme() {
        var savedTheme = localStorage.getItem('crawler-theme') || 'light';
        applyTheme(savedTheme);
        console.log('[theme.js] Applied saved theme:', savedTheme);
    }

    // Initialize theme toggle controls when DOM is ready
    function initThemeToggle() {
        console.log('[theme.js] Initializing theme toggle...');

        var themeCheckbox = document.getElementById('theme-checkbox');
        var currentTheme = localStorage.getItem('crawler-theme') || 'light';

        // Set initial checkbox state
        if (themeCheckbox) {
            themeCheckbox.checked = (currentTheme === 'dark');
            console.log('[theme.js] Checkbox found, checked:', themeCheckbox.checked);

            // Listen for checkbox change
            themeCheckbox.addEventListener('change', function(e) {
                console.log('[theme.js] Checkbox change event fired');
                var newTheme = this.checked ? 'dark' : 'light';

                applyTheme(newTheme);
                localStorage.setItem('crawler-theme', newTheme);
                console.log('[theme.js] Theme changed to:', newTheme);
            });
        }

        console.log('[theme.js] Initialization complete');
    }

    // Make toggleTheme globally available
    window.toggleTheme = toggleTheme;

    // Apply theme immediately (before DOM ready) to prevent flash
    initTheme();

    // Run toggle init on DOMContentLoaded
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initThemeToggle);
    } else {
        initThemeToggle();
    }
})();
