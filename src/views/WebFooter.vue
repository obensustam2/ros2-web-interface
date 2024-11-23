<template>
    <footer>
        <p>{{ footerText }}</p>
    </footer>
</template>

<script>
    export default {
        data() {
            return {
                footerText: 'Copyright 2024',
            }
        }
    }
</script>


<style scoped>
    footer {
        background-color: lightcyan;
        padding: 10px;
        text-align: center;
    }
</style>